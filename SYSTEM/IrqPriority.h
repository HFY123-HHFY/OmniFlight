#ifndef __IRQ_PRIORITY_H
#define __IRQ_PRIORITY_H

/*
 * IrqPriority.h — 四轴飞控中断优先级统一管理
 *
 * 设计原则：
 * - 数字越小，优先级越高（NVIC 标准语义）
 * - 策略集中在这里，Core 层只接受优先级参数而不做决策
 *
 * STM32F407: Cortex-M4, 4bit NVIC, 优先级 0~15
 */

#define IRQ_PRIO_MPU6050     1U   /* 最高实时：姿态传感器       */
#define IRQ_PRIO_TIM_CTRL    2U   /* 高实时：1ms 控制节拍        */
#define IRQ_PRIO_TIM_AUX     3U   /* 中实时：慢任务调度          */
#define IRQ_PRIO_USART       4U   /* 低实时：串口通信             */
#define IRQ_PRIO_DEFAULT     5U   /* 最低：缺省中断               */

#define IRQ_SUB_PRIO_MPU6050 0U

#endif /* __IRQ_PRIORITY_H */
