/*
 * BSP/Dshot/Dshot.c — DShot300 油门协议驱动
 *
 * 分层设计：
 * - Core 层提供通用 PWM/TIM/DMA 寄存器操作
 * - BSP/Dshot 层负责 DShot 协议逻辑（编码、帧构建、发送序列）
 * - 引脚映射由 Enroll/407_hw_config.h 的 HW_DSHOT_MOTOR_MAP 决定
 *
 * 依赖链：Enroll.h → 407_hw_config.h → f407_gpio.h
 *         + f407_pwm.h + f407_dma.h
 */

#include "Dshot.h"
#include "Enroll.h"        /* → 407_hw_config.h → HW_DSHOT_MOTOR_MAP */
#include "f407_pwm.h"      /* Core PWM: ConfigPin / InitTimer / SetCCR / GetTimerBase / ConfigDMABurst / EnableUpdateDMA / ForceUpdate */
#include "f407_dma.h"      /* Core DMA: EnableClock / StreamInit / SetTransfer / Enable / Disable / ClearFlags / WaitForComplete */

/* ===================================================================
 * F407 高级定时器寄存器视图（局部定义，用于 DShot 协议序列中
 * 需要直接操作 CCR/EGR 等寄存器的场景，避免层层封装影响性能）
 * =================================================================== */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} DShot_TimerRegs_t;

/* ===================================================================
 * DShot300 协议常量
 * =================================================================== */

/* 定时器周期 560 tick 时：75% = 420 tick（bit 1），37.5% = 210 tick（bit 0） */
#define DSHOT_BIT_1_TICKS          420U
#define DSHOT_BIT_0_TICKS          210U
#define DSHOT_FRAME_BITS           16U
#define DSHOT_FRAME_BUF_LEN        20U    /* 16 位数据 + 4 位帧尾间隔 */

/* TIM1 300kHz: ARR=560, PSC=1 → 168MHz/(1*560) ≈ 300kHz */
#define DSHOT_TIM_ARR              560U
#define DSHOT_TIM_PSC              1U

/* DMA: TIM1_UP 固定映射到 DMA2 Stream5 Channel6（F407 硬件决定） */
#define DSHOT_DMA_STREAM_BASE      F407_DMA2_STREAM5_BASE
#define DSHOT_DMA_STREAM_NUM       5U
#define DSHOT_DMA_CHANNEL          6U

/* ===================================================================
 * 静态缓冲区
 * =================================================================== */

/* DShot 帧缓冲区：[20 帧][4 通道]，每帧 DMA burst 一次更新 CCR1~CCR4 */
static uint16_t g_dshot_cmd[DSHOT_FRAME_BUF_LEN][4] = {{0}};

/* 物理通道油门值（CH1~CH4 顺序） */
static uint16_t g_dshot_throttle[4] = {0, 0, 0, 0};

/* 逻辑电机油门值（m1~m4 顺序） */
static uint16_t g_dshot_logical[4] = {0, 0, 0, 0};

/* TIM1 基地址指针（Init 时缓存，协议序列中直接使用） */
static DShot_TimerRegs_t *g_dshot_tim = 0;

/* ===================================================================
 * 协议内部函数
 * =================================================================== */

/*
 * 规范化 DShot 输入值：
 * - 0 表示停转
 * - 1~47 为保留命令区，统一按 0（停转）处理
 * - 48~2047 为有效油门区间
 */
static uint16_t DShot_NormalizeThrottle(uint16_t throttle)
{
    if (throttle == 0U)
    {
        return 0U;
    }

    if (throttle < DSHOT_THROTTLE_MIN)
    {
        return 0U;
    }

    if (throttle > DSHOT_THROTTLE_MAX)
    {
        return DSHOT_THROTTLE_MAX;
    }

    return throttle;
}

/*
 * 逻辑电机号 (m1~m4) → TIM1 物理通道映射。
 * 当前接线对应关系：CH1←m3, CH2←m1, CH3←m2, CH4←m4
 */
static void DShot_MapLogicalToChannels(uint16_t m1, uint16_t m2,
                                       uint16_t m3, uint16_t m4)
{
    g_dshot_throttle[0] = DShot_NormalizeThrottle(m3);
    g_dshot_throttle[1] = DShot_NormalizeThrottle(m1);
    g_dshot_throttle[2] = DShot_NormalizeThrottle(m2);
    g_dshot_throttle[3] = DShot_NormalizeThrottle(m4);
}

/*
 * DShot 数据包编码：
 * [15:5] 11 位油门值
 * [4]    1 位遥测请求（当前固定 0）
 * [3:0]  4 位 CRC 校验
 *
 * CRC4 = (packet ^ (packet>>4) ^ (packet>>8)) & 0x0F
 */
static uint16_t DShot_PacketEncode(uint16_t throttle, uint8_t telemetry)
{
    uint16_t packet;
    uint8_t crc;

    throttle = DShot_NormalizeThrottle(throttle);

    packet = (uint16_t)(((throttle & 0x07FFU) << 1U) | (telemetry ? 1U : 0U));
    crc = (uint8_t)((packet ^ (packet >> 4U) ^ (packet >> 8U)) & 0x0FU);
    return (uint16_t)((packet << 4U) | crc);
}

/*
 * 通过 DMA burst 将 4 路 DShot 数据包以位流形式发送到 TIM1 CCR1~CCR4。
 * 每个 DShot 帧（16 位）展开为 16 个 PWM 占空比值，由 DMA burst 逐个送入 CCR。
 */
static void DShot_SendFrame(uint16_t m1, uint16_t m2,
                            uint16_t m3, uint16_t m4)
{
    uint16_t packet1 = DShot_PacketEncode(m1, 0U);
    uint16_t packet2 = DShot_PacketEncode(m2, 0U);
    uint16_t packet3 = DShot_PacketEncode(m3, 0U);
    uint16_t packet4 = DShot_PacketEncode(m4, 0U);
    uint8_t i;

    /* 填充 16 位 DShot 数据帧 */
    for (i = 0U; i < DSHOT_FRAME_BITS; i++)
    {
        g_dshot_cmd[i][0] = ((packet1 >> (15U - i)) & 0x01U) ? DSHOT_BIT_1_TICKS : DSHOT_BIT_0_TICKS;
        g_dshot_cmd[i][1] = ((packet2 >> (15U - i)) & 0x01U) ? DSHOT_BIT_1_TICKS : DSHOT_BIT_0_TICKS;
        g_dshot_cmd[i][2] = ((packet3 >> (15U - i)) & 0x01U) ? DSHOT_BIT_1_TICKS : DSHOT_BIT_0_TICKS;
        g_dshot_cmd[i][3] = ((packet4 >> (15U - i)) & 0x01U) ? DSHOT_BIT_1_TICKS : DSHOT_BIT_0_TICKS;
    }

    /* 帧尾间隔：4 位全 0，让电调可靠分帧 */
    for (i = DSHOT_FRAME_BITS; i < DSHOT_FRAME_BUF_LEN; i++)
    {
        g_dshot_cmd[i][0] = 0U;
        g_dshot_cmd[i][1] = 0U;
        g_dshot_cmd[i][2] = 0U;
        g_dshot_cmd[i][3] = 0U;
    }

    /* 关闭上一次 DMA（如果还在运行） */
    F407_DMA_StreamDisable(DSHOT_DMA_STREAM_BASE);

    /* 清除标志，重设地址和计数，启动 DMA */
    F407_DMA_ClearStreamFlags(DSHOT_DMA_STREAM_NUM);
    F407_DMA_StreamSetTransfer(DSHOT_DMA_STREAM_BASE,
                               (uint32_t)(&g_dshot_tim->DMAR),
                               (uint32_t)g_dshot_cmd,
                               (uint16_t)(DSHOT_FRAME_BUF_LEN * 4U));
    F407_DMA_StreamEnable(DSHOT_DMA_STREAM_BASE);

    /* 等待 DMA 传输完成 */
    F407_DMA_WaitForComplete(DSHOT_DMA_STREAM_NUM);

    /* 关闭 DMA 并清除标志 */
    F407_DMA_StreamDisable(DSHOT_DMA_STREAM_BASE);
    F407_DMA_ClearStreamFlags(DSHOT_DMA_STREAM_NUM);

    /* 帧完成后拉低输出，防止保持上一次占空比 */
    g_dshot_tim->CCR1 = 0U;
    g_dshot_tim->CCR2 = 0U;
    g_dshot_tim->CCR3 = 0U;
    g_dshot_tim->CCR4 = 0U;
    g_dshot_tim->EGR |= 1U;  /* 触发更新，同步 CCR 到影子寄存器 */
}

/* ===================================================================
 * 公开接口
 * =================================================================== */

/*
 * DShot300 初始化。
 *
 * 流程：
 * 1) 从 hw_config 读取电机引脚映射，配置 GPIO 复用功能
 * 2) 配置 TIM1 为 300kHz PWM 基波（4 通道 PWM 模式 1）
 * 3) 初始化 DMA2 Stream5（TIM1_UP 触发 burst 传输）
 */
void DShot_Init(void)
{
    uint32_t timBase;

    /* ---- 1) 引脚配置：从 hw_config 读取 ---- */
#define DSHOT_CFG_PIN(ch, port, pin) \
    F407_PWM_ConfigPin((void *)(port), (uint16_t)(pin), HW_DSHOT_TIM_ID);
    HW_DSHOT_MOTOR_MAP(DSHOT_CFG_PIN)
#undef DSHOT_CFG_PIN

    /* ---- 2) TIM1 300kHz PWM 初始化 ---- */
    F407_PWM_InitTimer(HW_DSHOT_TIM_ID,
                       (uint16_t)(DSHOT_TIM_ARR - 1U),
                       (uint16_t)(DSHOT_TIM_PSC - 1U));

    /* 配置 4 个通道为 PWM 模式 1 + 预装载，初始占空比 0 */
    F407_PWM_SetCCR(HW_DSHOT_TIM_ID, 1U, 0U);
    F407_PWM_SetCCR(HW_DSHOT_TIM_ID, 2U, 0U);
    F407_PWM_SetCCR(HW_DSHOT_TIM_ID, 3U, 0U);
    F407_PWM_SetCCR(HW_DSHOT_TIM_ID, 4U, 0U);

    /* 缓存 TIM1 基地址，协议序列中用于直接 CCR/EGR 访问 */
    timBase = F407_PWM_GetTimerBase(HW_DSHOT_TIM_ID);
    g_dshot_tim = (DShot_TimerRegs_t *)timBase;

    /* ---- 3) DMA2 Stream5 初始化（TIM1_UP → DMA burst → CCR1~CCR4）---- */

    /* 使能 DMA2 时钟 */
    F407_DMA_EnableClock();

    /* 初始化 DMA 流：CHSEL=6(TIM1_UP), M2P, MINC, 16-bit, 高优先级 */
    F407_DMA_StreamInit(DSHOT_DMA_STREAM_BASE,
                        DSHOT_DMA_CHANNEL,
                        F407_DMA_CR_DIR_M2P
                        | F407_DMA_CR_MINC
                        | F407_DMA_CR_PSIZE_16
                        | F407_DMA_CR_MSIZE_16
                        | F407_DMA_CR_PL_HIGH);

    /* 清除 DMA 流中断标志 */
    F407_DMA_ClearStreamFlags(DSHOT_DMA_STREAM_NUM);

    /* 配置 TIM1 DMA burst：每次 Update 传输 CCR1~CCR4 共 4 个 16-bit 值 */
    F407_PWM_ConfigDMABurst(HW_DSHOT_TIM_ID,
                            0x0DU,  /* DBA = 0x34(CCR1偏移) / 4 = 0x0D */
                            3U);    /* DBL = 3 → 4 次传输 */

    /* 使能 TIM1 Update DMA 请求 */
    F407_PWM_EnableUpdateDMA(HW_DSHOT_TIM_ID);
}

/*
 * 一次性发送 4 路电机的 DShot 油门数据。
 * 入参为逻辑电机号 m1~m4，内部映射到 TIM1 物理通道后发送。
 */
void DShot_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    /* 若未初始化则直接返回 */
    if (g_dshot_tim == 0)
    {
        return;
    }

    g_dshot_logical[0] = DShot_NormalizeThrottle(m1);
    g_dshot_logical[1] = DShot_NormalizeThrottle(m2);
    g_dshot_logical[2] = DShot_NormalizeThrottle(m3);
    g_dshot_logical[3] = DShot_NormalizeThrottle(m4);

    DShot_MapLogicalToChannels(g_dshot_logical[0],
                               g_dshot_logical[1],
                               g_dshot_logical[2],
                               g_dshot_logical[3]);

    DShot_SendFrame(g_dshot_throttle[0],
                    g_dshot_throttle[1],
                    g_dshot_throttle[2],
                    g_dshot_throttle[3]);
}
