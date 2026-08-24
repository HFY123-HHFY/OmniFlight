#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 四轴飞控中断优先级统一管理（FreeRTOS 版）
 *
 * STM32F407: Cortex-M4, 4bit NVIC, 优先级 0~15（数值越小越高）。
 *
 * FreeRTOS configMAX_SYSCALL_INTERRUPT_PRIORITY = 5：
 *   优先级 5~15 → 内核"感知"，会被临界区短暂屏蔽，允许调用 ...FromISR()。
 *   优先级 0~4  → 内核"不感知"，永不被屏蔽，但严禁调用任何 FreeRTOS API。
 *
 * 分配：
 *  - TIM_CTRL(5)：500Hz 控制节拍（TIM3），给 ControlTask 发二进制信号量 → 必须 >= 5
 *  - MPU6050(6) ：EXTI 数据就绪，给 SensorTask 发二进制信号量         → 必须 >= 5
 *  - USART(4)   ：异步 TX/RX 环形队列，纯内存操作不调 RTOS API，
 *                 设为"不感知"(4 < 5)，永不被内核屏蔽 → 串口零丢包，异步打印不死锁
 *  - 内核：SysTick/PendSV 固定 15（最低），由 FreeRTOS 自身接管
 */

#define IRQ_PRIO_MPU6050     6U   /* MPU6050 EXTI 数据就绪（感知，给传感器信号量） */
#define IRQ_PRIO_TIM_CTRL    5U   /* TIM3 控制节拍 500Hz（感知，给控制信号量） */
#define IRQ_PRIO_TIM_AUX     5U   /* TIM2（RTOS 后不再用作任务分频，保留兼容） */
#define IRQ_PRIO_USART       4U   /* 串口（不感知，异步 TX/RX 永不被屏蔽） */
#define IRQ_PRIO_DEFAULT     5U   /* 缺省中断 */

#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* __IRQ_PRIORITY_H */