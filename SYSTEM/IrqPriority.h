#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 四轴飞控中断优先级统一管理
 *
 * 设计原则：
 * - 数字越小，优先级越高（NVIC 标准语义）
 * - 策略集中在这里，Core 层只接受优先级参数而不做决策
 *
 * 飞控优先级分配：
 *  ┌──────────────────────────────────────────┐
 *  │  0  SysTick / 系统时钟                    │
 *  │  1  MPU6050 EXTI                          │ ← 姿态数据就绪，DMP FIFO 防溢出
 *  │  2  TIM3 控制节拍 (1ms)                   │ ← PID/混控 500Hz 心跳
 *  │  3  TIM2 慢任务节拍 (1ms)                 │ ← NRF/QMC/BMP/printf 调度中心
 *  │  4  USART (×2)                            │ ← 通信丢包可重传
 *  │  5+ 缺省                                  │
 *  └──────────────────────────────────────────┘
 *
 * 注：
 * - 所有 ISR 只做计数+置标志，不执行耗时运算
 * - PID 计算在 main loop 中执行，不在 ISR 里
 * - NRF24L01 为 SPI 轮询模式，不使用中断
 * - DShot 使用 DMA 轮询（阻塞发送），无 DMA 中断
 *
 * NVIC 差异：
 * - STM32F407: Cortex-M4, 4bit NVIC, 0~15
 * - STM32F103: Cortex-M3, 4bit NVIC, 0~15
 * - MSPM0G3507: Cortex-M0+, 2bit NVIC, 0~3
 */

/* ================================================================
 *  G3507：2 bit 优先级 (0~3)，需要压缩映射
 * ================================================================ */
#if (ENROLL_MCU_TARGET == ENROLL_MCU_G3507)

#define IRQ_PRIO_MPU6050     0U   /* 最高：姿态传感器            */
#define IRQ_PRIO_TIM_CTRL    1U   /* 高：1ms 控制节拍            */
#define IRQ_PRIO_TIM_AUX     1U   /* 合并到 TIM_CTRL（2bit 只有 4 级） */
#define IRQ_PRIO_USART       2U   /* 中：串口通信              */
#define IRQ_PRIO_DEFAULT     3U   /* 低：未指定中断            */

#define IRQ_SUB_PRIO_MPU6050 0U

/* ================================================================
 *  F103 / F407：4 bit 优先级 (0~15)
 * ================================================================ */
#else

#define IRQ_PRIO_MPU6050     1U   /* 最高实时：姿态传感器       */
#define IRQ_PRIO_TIM_CTRL    2U   /* 高实时：1ms 控制节拍        */
#define IRQ_PRIO_TIM_AUX     3U   /* 中实时：慢任务调度          */
#define IRQ_PRIO_USART       4U   /* 低实时：串口通信             */
#define IRQ_PRIO_DEFAULT     5U   /* 最低：缺省中断               */

#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* ENROLL_MCU_TARGET */

#endif /* __IRQ_PRIORITY_H */
