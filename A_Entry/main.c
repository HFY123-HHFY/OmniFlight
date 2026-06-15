/*
* OmniFlight - 四轴飞控
*/

/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"

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
#include "QMC5883P.h"
#include "BMP280.h"
#include "Dshot.h" /* DShot协议 初始化 */
#include "NRF24L01.h"

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_USART_Register();				/* USART 资源注册 */
	/* Enroll_PWM_Register() 不再需要：电机输出由 BSP/Dshot 管理 */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 		/* USART 中断回调：注册时自动使能异步 TX */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task1_Callback);           	/* TIM1: PID */
	API_TIM_RegisterIrqHandler(API_TIM2, Control_Task2_Callback);               /* TIM2: printf/时间戳 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，波特率 115200
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3，波特率 115200

	API_TIM_Init(API_TIM1, 1U); /* TIM1: PID 节拍，每 1ms */
	API_TIM_Init(API_TIM2, 1U); /* TIM2: printf/时间戳  每 1ms */
	

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	App_I2C_ScanOnce();					/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW); // 初始化LED-低电平
	MPU_Init();	/* 初始化MPU6050 */
	uint8_t mpu6050_dma_int = mpu_dmp_init(); /* 初始化MPU6050 DMP */
	usart_printf(USART1, "mpu6050_dma_int= %d\r\n", mpu6050_dma_int);
	Enroll_MPU6050_Register();				/* MPU6050 INT 资源注册（DMP 初始化后才能使能 EXTI） */
	QMC_Init();		/* 初始化QMC5883P */
	BMP280Init();	/* 初始化BMP280 */
	DShot_Init();	/* 初始化DShot协议 */
	NRF24L01_Init();	/* 初始化NRF24L01 */

	while (1)
	{
	/* MPU6050 DMP */
		mpu_angle();

	 /* 串级PID控制 - 2ms 姿态环*/
		if (pid_task_flag != 0U)   // 500Hz 姿态环
		{
			pid_task_flag = 0U;
			// PID_Pitch_Roll_Combined(Pitch, Roll);
		}
	/* 串口数据打印 */
		if (print_task_flag != 0U)
		{
			print_task_flag = 0U;
			// usart_printf(USART1, "Timer_Bsp_t: %lu\r\n", Timer_Bsp_t);
			usart_printf(USART1, "Pitch=%.2f Roll=%.2f Yaw=%.2f\r\n", Pitch, Roll, Yaw);
			// usart_printf(USART2, "Pitch=%.2f Roll=%.2f Yaw=%.2f\r\n", Pitch, Roll, Yaw); /* 无线串口 */
		}
	}
}
