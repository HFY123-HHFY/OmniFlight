#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include "tim.h"
#include "usart.h"

extern uint32_t Timer_Bsp_t; // 程序运行的时间戳（s）
extern volatile uint8_t print_task_flag; // printf节拍-50ms

void Control_Task1_Callback(API_TIM_Id_t id);           /* TIM1: PID 控制节拍 */
void Control_Task2_Callback(API_TIM_Id_t id);           /* TIM2: printf/时间戳 */

/*
 * USART 中断回调。
 * 由 USART ISR 触发，内部调度 usart_irq_dispatch_by_id 处理 TX 队列排空和 RX 接收。
 * 必须在 main.c 中通过 Enroll_USART_RegisterIrqHandler 注册，否则 TX 队列不会被排空，
 * 会导致 TXE 中断无限循环卡死程序。
 */
void Control_Task_USART_Callback(API_USART_Id_t id);

#endif /* __CONTROL_TASK_H */
