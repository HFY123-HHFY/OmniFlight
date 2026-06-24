#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "Dshot.h"
#include "Motor.h"
#include "MPU6050.h"
#include "IMU.h"

/* =========================================================================
 * 任务标志位
 *
 * 全部由定时器 ISR 置位（1），主循环消费后清零（0）。
 * 统一模式：ISR 管时间 → 置标志 → 主循环管执行
 * ========================================================================= */

/* 程序运行的时间戳（s），TIM2 每 1s 递增 */
uint32_t Timer_Bsp_t = 0;

/* TIM2 分发：慢速外设任务 */
volatile uint8_t nrf_task_flag   = 0U;   /* 100Hz NRF24L01 遥控通信 */
volatile uint8_t qmc_task_flag   = 0U;   /* 50Hz  QMC5883P 磁力计   */
volatile uint8_t bmp_task_flag   = 0U;   /* 20Hz  BMP280 气压计     */
volatile uint8_t print_task_flag = 0U;   /* 10Hz  串口打印          */

/* =========================================================================
 * Control_Task1_Callback — TIM1 (TIM3) 1ms ISR
 *
 * 职责：飞控心脏节拍
 *   - PID 控制 + 电机输出：500Hz（每 2ms）
 * 
 * ========================================================================= */
void Control_Task1_Callback(API_TIM_Id_t id)
{
	static uint8_t  pid_2ms_tick = 0U;

	if (id != API_TIM1)
	{
		return;
	}

	pid_2ms_tick++;

	if (pid_2ms_tick >= 2U)
	{
		pid_2ms_tick = 0U;
		/* IMU 偏航融合：陀螺积分 (500Hz) */
		IMU_Yaw_IntegrateGyro((float)gyroz / GYRO_SENS_2000DPS, 0.002f);
		if (Key == 1)
		{
			// PID_Pitch_Roll_Combined(Pitch, Roll);  /* PID → 混控 → DShot_Write */
		}
		else
		{
			Motor_Test();  /* 未解锁时仍走电机状态机（处理掉电缓降） */
		}
	}
}

/* =========================================================================
 * Control_Task2_Callback — TIM2 1ms ISR
 *
 * 职责：所有非飞控周期任务的统一调度中心
 *
 *   1000ms → Timer_Bsp_t++     (1Hz  时间戳)
 *    100ms → print_task_flag   (10Hz 串口打印)
 *     50ms → bmp_task_flag     (20Hz 气压计)
 *     20ms → qmc_task_flag     (50Hz 磁力计)
 *     10ms → nrf_task_flag     (100Hz 遥控通信)
 *
 * 所有计数器和标志位集中在此，改频率只需改这若干个数字。
 * ========================================================================= */
void Control_Task2_Callback(API_TIM_Id_t id)
{
	static uint8_t nrf_tick    = 0U;
	static uint8_t qmc_tick    = 0U;
	static uint8_t bmp_tick    = 0U;
	static uint8_t printf_tick = 0U;
	static uint16_t time_t     = 0U;

	if (id != API_TIM2)
	{
		return;
	}

	/* ---- NRF24L01: 100Hz (每 10ms) ---- */
	nrf_tick++;
	if (nrf_tick >= 10U)
	{
		nrf_tick = 0U;
		nrf_task_flag = 1U;
	}

	/* ---- QMC5883P: 50Hz (每 20ms) ---- */
	qmc_tick++;
	if (qmc_tick >= 20U)
	{
		qmc_tick = 0U;
		qmc_task_flag = 1U;
	}

	/* ---- BMP280: 20Hz (每 50ms) ---- */
	bmp_tick++;
	if (bmp_tick >= 50U)
	{
		bmp_tick = 0U;
		bmp_task_flag = 1U;
	}

	/* ---- printf: 10Hz (每 100ms) ---- */
	printf_tick++;
	if (printf_tick >= 100U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	/* ---- 时间戳: 1Hz (每 1000ms) ---- */
	time_t++;
	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

/* =========================================================================
 * Control_Task_USART_Callback — USART 中断回调
 *
 * 由 USART ISR 触发，调度 TX 队列排空（printf 异步发送的核心）。
 * 传 NULL 跳过 RX 接收处理，仅做 TX 排空。
 * 此回调必须注册（Enroll_USART_RegisterIrqHandler），否则 TXE 中断会无限循环。
 * ========================================================================= */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	usart_irq_dispatch_by_id(id, 0, 0);
}
