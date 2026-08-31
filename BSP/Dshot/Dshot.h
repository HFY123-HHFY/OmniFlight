#ifndef __DSHOT_H
#define __DSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DShot300 油门协议驱动（BSP 层）
 *
 * 职责：
 * - DShot 数据包编码（11-bit 油门 + 1-bit 遥测 + 4-bit CRC）
 * - DShot 帧位流展开与 DMA burst 发送
 * - 逻辑电机号 → 物理通道映射
 *
 * 依赖：
 * - Core/f407_pwm：定时器/PWM 配置
 * - Core/f407_dma：DMA 流控制
 * - Enroll/407_hw_config.h：电机引脚映射（HW_DSHOT_MOTOR_MAP）
 *
 * DShot300 参数：TIM1 168MHz → 300kHz PWM（ARR=560, PSC=1）
 * 位宽：75% 占空比 = 逻辑 1，37.5% 占空比 = 逻辑 0
 */

/* DShot 油门定义：0 停转, 1~47 为命令区, 48~2047 为正常油门 */
#define DSHOT_THROTTLE_MIN           48U
#define DSHOT_THROTTLE_MAX           2047U
#define PWM_DUTY_MAX                 ((uint16_t)DSHOT_THROTTLE_MAX)

/* ─────────────────────────────────────────────────────────────────
 * ★ 电机方位映射表（调机时只改这里！）
 *
 * 逻辑电机 m1~m4 → TIM1 物理通道 CH1~CH4
 *
 * TIM1 通道对应引脚（见 Enroll/407_hw_config.h 的 HW_DSHOT_MOTOR_MAP）：
 *   CH1 = PE9   CH2 = PE11   CH3 = PE13   CH4 = PE14
 *
 * 调试方法：逐个测试 DShot_Write(mX, 0, 0, 0) 看哪个电机转，
 * 把 DSHOT_Mx_CH 改成该电机实际所接的通道号即可。
 * ───────────────────────────────────────────────────────────────── */
#define DSHOT_M1_CH                  3U
#define DSHOT_M2_CH                  4U
#define DSHOT_M3_CH                  1U
#define DSHOT_M4_CH                  2U

/*
 * DShot300 初始化。
 * 从 hw_config 读取电机引脚映射，配置 TIM1 + DMA2 Stream5。
 */
void DShot_Init(void);

/*
 * 一次性发送 4 路电机的 DShot 油门数据。
 * m1~m4: 逻辑电机号，内部映射到 TIM1 物理通道。
 */
void DShot_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#ifdef __cplusplus
}
#endif

#endif /* __DSHOT_H */
