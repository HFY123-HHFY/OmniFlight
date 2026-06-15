#ifndef __F407_DMA_H
#define __F407_DMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * F407 DMA 控制器与流寄存器视图。
 * 公开结构体以便 BSP 层（如 DShot）进行寄存器级操作。
 */

/* DMA 流寄存器 */
typedef struct
{
    volatile uint32_t CR;    /* 配置寄存器 */
    volatile uint32_t NDTR;  /* 传输数量寄存器 */
    volatile uint32_t PAR;   /* 外设地址寄存器 */
    volatile uint32_t M0AR;  /* 存储器 0 地址寄存器 */
    volatile uint32_t M1AR;  /* 存储器 1 地址寄存器 */
    volatile uint32_t FCR;   /* FIFO 控制寄存器 */
} F407_DMA_StreamRegs_t;

/* DMA 控制器寄存器 */
typedef struct
{
    volatile uint32_t LISR;  /* 低中断状态寄存器（Stream 0~3） */
    volatile uint32_t HISR;  /* 高中断状态寄存器（Stream 4~7） */
    volatile uint32_t LIFCR; /* 低中断标志清除寄存器 */
    volatile uint32_t HIFCR; /* 高中断标志清除寄存器 */
} F407_DMA_CtrlRegs_t;

/* ====== 基地址 ====== */

#define F407_DMA2_BASE              (0x40026400UL)
#define F407_DMA2                   ((F407_DMA_CtrlRegs_t *)F407_DMA2_BASE)

/* Stream 基地址：DMA2_BASE + 0x10 + streamNum * 0x18 */
#define F407_DMA2_STREAM0_BASE      (0x40026410UL)
#define F407_DMA2_STREAM1_BASE      (0x40026428UL)
#define F407_DMA2_STREAM2_BASE      (0x40026440UL)
#define F407_DMA2_STREAM3_BASE      (0x40026458UL)
#define F407_DMA2_STREAM4_BASE      (0x40026470UL)
#define F407_DMA2_STREAM5_BASE      (0x40026488UL)
#define F407_DMA2_STREAM6_BASE      (0x400264A0UL)
#define F407_DMA2_STREAM7_BASE      (0x400264B8UL)

/* ====== CR 控制位 ====== */

#define F407_DMA_CR_EN              (1UL << 0U)
#define F407_DMA_CR_DIR_M2P         (1UL << 6U)   /* 存储器到外设 */
#define F407_DMA_CR_DIR_P2M         (0UL << 6U)   /* 外设到存储器 */
#define F407_DMA_CR_MINC            (1UL << 10U)  /* 存储器地址自增 */
#define F407_DMA_CR_PSIZE_8         (0UL << 11U)
#define F407_DMA_CR_PSIZE_16        (1UL << 11U)
#define F407_DMA_CR_PSIZE_32        (2UL << 11U)
#define F407_DMA_CR_MSIZE_8         (0UL << 13U)
#define F407_DMA_CR_MSIZE_16        (1UL << 13U)
#define F407_DMA_CR_MSIZE_32        (2UL << 13U)
#define F407_DMA_CR_PL_LOW          (0UL << 16U)
#define F407_DMA_CR_PL_MEDIUM       (1UL << 16U)
#define F407_DMA_CR_PL_HIGH         (2UL << 16U)
#define F407_DMA_CR_PL_VERYHIGH     (3UL << 16U)
#define F407_DMA_CR_CHSEL_POS       (25U)

/* ====== HISR / HIFCR 中断标志位 ====== */

/* 根据 Stream 编号获取 TCIF 标志位 */
#define F407_DMA_TCIF(stream)       (1UL << ((stream) >= 4U ? (((stream) - 4U) * 6U + 16U) : ((stream) * 6U)))
#define F407_DMA_HTIF(stream)       (1UL << ((stream) >= 4U ? (((stream) - 4U) * 6U + 18U) : ((stream) * 6U + 2U)))
#define F407_DMA_TEIF(stream)       (1UL << ((stream) >= 4U ? (((stream) - 4U) * 6U + 20U) : ((stream) * 6U + 4U)))
#define F407_DMA_DMEIF(stream)      (1UL << ((stream) >= 4U ? (((stream) - 4U) * 6U + 21U) : ((stream) * 6U + 5U)))
#define F407_DMA_FEIF(stream)       (1UL << ((stream) >= 4U ? (((stream) - 4U) * 6U + 22U) : ((stream) * 6U + 6U)))
#define F407_DMA_CLR_MASK(stream)   (F407_DMA_TCIF(stream) | F407_DMA_HTIF(stream) | \
                                     F407_DMA_TEIF(stream) | F407_DMA_DMEIF(stream) | F407_DMA_FEIF(stream))

/* ====== 公开函数 ====== */

/* 使能 DMA2 时钟。 */
void F407_DMA_EnableClock(void);

/*
 * 初始化 DMA 流：
 * - 禁用流（若已使能）
 * - 清除所有中断标志
 * - 配置基本参数
 * configFlags 由以下宏组合：CR_DIR_M2P/P2M | CR_MINC | CR_PSIZE_* | CR_MSIZE_* | CR_PL_*
 */
void F407_DMA_StreamInit(uint32_t streamBase, uint8_t channel,
                         uint32_t configFlags);

/*
 * 配置 DMA 传输地址和长度。
 */
void F407_DMA_StreamSetTransfer(uint32_t streamBase,
                                uint32_t periphAddr,
                                uint32_t memAddr,
                                uint16_t count);

/* 使能 DMA 流。 */
void F407_DMA_StreamEnable(uint32_t streamBase);

/* 禁用 DMA 流（轮询等待关闭完成）。 */
void F407_DMA_StreamDisable(uint32_t streamBase);

/* 查询 DMA 流是否已使能。 */
uint8_t F407_DMA_StreamIsEnabled(uint32_t streamBase);

/* 清除指定 Stream 的所有中断标志。 */
void F407_DMA_ClearStreamFlags(uint8_t streamNum);

/* 轮询等待指定 Stream 传输完成（TCIF 置位）。 */
void F407_DMA_WaitForComplete(uint8_t streamNum);

#ifdef __cplusplus
}
#endif

#endif /* __F407_DMA_H */
