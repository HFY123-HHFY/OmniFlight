#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "Dshot.h"

/* 程序运行的时间戳（s） */
uint32_t Timer_Bsp_t = 0;

/* printf节拍 */
volatile uint8_t print_task_flag = 0;

/*
 * 定时器回调函数：
 * 由 API_TIM 的通用中断分发层在更新中断到来后调用。
 * API_TIM1: 1ms -> PID 2ms
 */
void Control_Task1_Callback(API_TIM_Id_t id)
{
	static uint8_t pid_2ms_tick = 0U;

	if (id != API_TIM1)
	{
		return;
	}

	pid_2ms_tick++;

	if (pid_2ms_tick >= 2U)
	{
		pid_2ms_tick = 0U;
		pid_task_flag = 1U;
	}
}

/*
 * API_TIM2: 1ms -> Key + printf + time
 */
void Control_Task2_Callback(API_TIM_Id_t id)
{
	static uint8_t printf_tick = 0U;
	static uint16_t time_t = 0U;

	if (id != API_TIM2)
	{
		return;
	}

	printf_tick++;
	time_t++;

	if (printf_tick >= 100U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

/*
 * USART 中断回调。
 * 由 USART ISR 触发，调度 TX 队列排空（printf 异步发送的核心）。
 * 传 NULL 跳过 RX 接收处理，仅做 TX 排空。
 * 此回调必须注册（Enroll_USART_RegisterIrqHandler），否则 TXE 中断会无限循环。
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	usart_irq_dispatch_by_id(id, 0, 0);
}
