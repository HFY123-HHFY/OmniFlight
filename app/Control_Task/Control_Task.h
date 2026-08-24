#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include "tim.h"
#include "usart.h"

/*
 * Control_Task.h — RTOS 任务调度模块（替代裸机的前后台标志位调度）
 *
 * 任务模型（6 个任务，优先级 1~6，0 为 idle）：
 *   ControlTask(6)  — 500Hz 姿态控制，TIM3 信号量唤醒
 *   SensorTask(5)   — 200Hz MPU6050 DMP 读取，EXTI 信号量唤醒
 *   LidarTask(4)    — STP-23L 激光雷达协议解析，vTaskDelayUntil 轮询
 *   RadioTask(3)    — 100Hz NRF24L01 遥控+遥测，vTaskDelayUntil
 *   TelemetryTask(2)— 10Hz 串口打印，vTaskDelayUntil
 *   HealthTask(1)   — 1Hz 时间戳 + LED 心跳，vTaskDelay
 *
 * ISR 回调（仅做信号量/通知，不跑业务逻辑）：
 *   Control_Task1_Callback — TIM3 500Hz 给控制信号量
 *   Control_Task_USART_Callback — USART async TX/RX（保持 FreeRTOS 不感知，不调 RTOS API）
 *   ControlTask_NotifyMpuIsr — 由 MPU6050 EXTI 回调调用，给传感器信号量
 */

/* 程序运行时间戳 (s)，由 HealthTask 每秒递增。 */
extern uint32_t Timer_Bsp_t;

/* ── ISR 回调（由 main.c 注册到 Enroll/API 层）── */

/* TIM3 中断回调：500Hz 控制节拍 → 给 xControlSem 信号量。 */
void Control_Task1_Callback(API_TIM_Id_t id);

/* USART 中断回调：TX 队列排空 + RX 按串口分发（FreeRTOS 不感知，不调任何 RTOS API）。 */
void Control_Task_USART_Callback(API_USART_Id_t id);

/* 由 MPU6050_EXTI_Callback 调用：给传感器信号量（ISR 上下文，优先级 6，可调 FromISR）。 */
void ControlTask_NotifyMpuIsr(void);

/* ── RTOS 初始化（由 main.c 调用，在硬件外设启动前执行）── */

/*
 * 创建所有 FreeRTOS 对象（信号量 / 互斥锁 / 任务）。
 * 必须在 TIM3 和 MPU6050 EXTI 启动前调用，确保 ISR 给信号量时对象已存在。
 */
void Control_Task_RTOSInit(void);

#endif /* __CONTROL_TASK_H */