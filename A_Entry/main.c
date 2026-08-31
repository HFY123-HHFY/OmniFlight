/*
* OmniFlight — 四轴飞控 (FreeRTOS 版)
*
* 初始化相关外设并校准各个传感器：
* 陀螺仪、磁力计、气压计各五秒，共计飞控需静止10S左右-蓝灯亮
*
* 校准完毕蓝灯灭
* 全部外设初始化完-蜂鸣器鸣笛蓝灯灭
*
* 锁定油门-红灯亮
*/

/* 系统 & RTOS */
#include "sys.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"

/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "PID/PID.h"
#include "Control/Control.h"
#include "Control_Task/Control_Task.h"

/*BSP硬件抽象层*/
#include "LED.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "Motor.h"
#include "QMC5883P.h"
#include "BMP280.h"
#include "Dshot.h" /* DShot协议 初始化 */
#include "NRF24L01.h"
#include "Buzzer.h"
#include "IMU.h"
#include "Altitude.h"
#include "MTF02P.h"

int main(void)
{
	/* 系统时钟配置初始化 */
	SYS_Init();

	/* 注册层：注册相关资源，登记资源映射 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_USART_Register();				/* USART 资源注册 */
	Enroll_PWM_Register();					/* PWM 资源注册（Buzzer: TIM3 CH4） */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_NRF24L01_Register();				/* NRF24L01 CE 引脚注册 */

	/* 注册后绑定中断回调 */
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback);
	                                            /* USART 中断回调：TX 排空 + RX 分发 */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task1_Callback);
	                                            /* TIM2: 控制节拍 500Hz → 给信号量 */

	/* ═══════════════════════════════════════════════════════════════
	 * RTOS 初始化：创建信号量/互斥锁/任务。
	 * 必须在 TIM2 和 MPU6050 EXTI 启动前调用，确保 ISR 给信号量时对象已存在。
	 * ═══════════════════════════════════════════════════════════════ */
	Control_Task_RTOSInit();

	/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，波特率 115200U — 板载调试串口
	// API_USART_Init(API_USART2, 115200U); // 初始化 USART2，波特率 115200  — 板载调试串口 -预留
	// API_USART_Init(API_USART3, 115200U); // 初始化 USART3，波特率 115200  — 板载调试串口 -预留
	API_USART_Init(API_USART4, 115200U); // 初始化 USART4，波特率 115200  — MTF-02P

	// IMU_Init();			/* IMU 状态重置。静态变量默认已零初始化，ControlTask 会自动开始零偏采集 */
	API_TIM_Init(API_TIM1, 2U); /* TIM2: 控制节拍，每 2ms = 500Hz 直接给信号量 */
	API_PWM_Init(API_PWM_TIM3, (1000000U / 2700U) - 1, 84U - 1U); /* TIM3: 蜂鸣器 PWM，ARR=369，PSC=83，1MHz/2700Hz≈369，50%占空比 */

	/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

	/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW);	/* LED 初始化-低电平 */
	MPU_Init();	/* 初始化MPU6050 */
	uint8_t mpu6050_dma_int = mpu_dmp_init(); /* 初始化MPU6050 DMP */
	usart_printf(USART1, "mpu6050_dma_int= %d\r\n", mpu6050_dma_int);
	Enroll_MPU6050_Register();				/* MPU6050 INT 资源注册（DMP 初始化后才能使能 EXTI） */

	/* 校准过程中飞行器必须保持静止！LED3 亮 = 校准所有传感器中，灭 = 所有传感器校准完成 */
	LED_Control(LED3, LED_HIGH);
	/* 5秒陀螺零偏校准 */
	float gravity_ref = 0.0f;
	if (GyroBias_Calibrate(1000U, &gravity_ref) == 0U)
	{
		while (1) {}
	}
	/* 初始化QMC5883P */
	// QMC_Init();
	/* 高度融合初始化（5秒重力参考采集） */
	// Altitude_Init(gravity_ref);
	/* 初始化BMP280（5秒自动地面归零校准） */
	// BMP280Init();
	/* 初始化NRF24L01 */
	NRF24L01_Init();
	/* 初始化PID控制 */
	PID_Contorl_Init();
	/* 初始化DShot协议 */
	DShot_Init();
	/* 初始化MTF-02P光流测距协议解析 */
	MTF02P_Init();
	/* 所有外设初始化完成-蜂鸣器初始化 */
	Buzzer_Init();

	Set_PID(&pid_pitch,      4.0f, 0.0f, 0.20f);
	Set_PID(&pid_rate_pitch, 1.0f, 0.015f, 0.0f);

	Set_PID(&pid_roll,       4.0f, 0.0f, 0.20f);
	Set_PID(&pid_rate_roll,  1.0f, 0.015f, 0.0f);

	/* ═══════════════════════════════════════════════════════════════
	 * 启动 FreeRTOS 调度器 — 此后由 RTOS 接管 5 个任务，永不返回。
	 * ═══════════════════════════════════════════════════════════════ */
	vTaskStartScheduler();

	/* 永远不会到达这里 */
	for (;;) {}
}
