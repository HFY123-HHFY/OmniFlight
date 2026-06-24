/*
* OmniFlight - 四轴飞控 
* 初始化相关外设并校准各个传感器：
* 陀螺仪、磁力计、气压计各五秒，共计飞控需静止10S左右-蓝灯亮

* 校准完毕蓝灯灭
* 全部外设初始化完-蜂鸣器鸣笛蓝灯灭

* 锁定油门-红灯亮
*/

/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

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

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 		/* USART 中断回调：注册时自动使能异步 TX */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task1_Callback);           	/* TIM1: PID */
	API_TIM_RegisterIrqHandler(API_TIM2, Control_Task2_Callback);               /* TIM2: printf/时间戳 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1，波特率 115200
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3，波特率 115200

	IMU_Init();			/* IMU 偏航融合初始化（必须在 TIM1 之前，否则 ISR 用未初始化状态） */
	API_TIM_Init(API_TIM1, 1U); /* TIM1: PID 节拍，每 1ms */
	API_TIM_Init(API_TIM2, 1U); /* TIM2: printf/时间戳  每 1ms */
	API_PWM_Init(API_PWM_TIM3, (1000000U / 2700U) - 1, 84U - 1U);

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

	/* 校准过程中飞行器必须保持静止！LED3 亮 = 校准所以传感器中，灭 = 所以传感器校准完成 */
	LED_Control(LED3, LED_HIGH);
	/* 5秒陀螺零偏校准 */
	float gravity_ref = 0.0f;
	if (GyroBias_Calibrate(1000U, &gravity_ref) == 0U)
	{
		/* calib timeout - halt */
		while (1) {}
	}
	/* 初始化QMC5883P */
	QMC_Init();
	/* 高度融合初始化（5秒重力参考采集） */
	Altitude_Init(gravity_ref);
	/* 初始化BMP280（5秒自动地面归零校准） */
	BMP280Init();
	/* 初始化NRF24L01 */	
	NRF24L01_Init();	
	/* 初始化PID控制 */
	PID_Contorl_Init();	
	/* 初始化DShot协议 */
	DShot_Init();	
	/* 所有外设初始化完成-蜂鸣器初始化 */
	Buzzer_Init();	

	/*      
	 * 串级PID参数（基于 dt=0.002s，500Hz）
	 * 调参顺序：先 KP → 再 KD → 最后 KI
	 */
	Set_PID(&pid_pitch,      4.0f, 0.3f, 0.0f);
	Set_PID(&pid_rate_pitch, 1.8f, 0.0f, 0.015f);

	Set_PID(&pid_roll,       4.0f, 0.3f, 0.0f);
	Set_PID(&pid_rate_roll,  1.8f, 0.0f, 0.015f);

/* ── 调试开关：起飞前设 0，关闭所有 printf ── */
#define DEBUG_PRINT_ENABLE  0U

	while (1)
	{
		/* 
		*MPU6050数据读取（陀螺仪 + DMP 姿态）*/
		if (mpu_flag == 1U)
		{
			mpu_flag = 0U;
			mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
			MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
			MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
		}

		/* 
		* 遥控链路 (100Hz)  遥控输入 + 遥测回传*/
		if (nrf_task_flag != 0U)
		{
			nrf_task_flag = 0U;
			NRF24L01_Data();
		}

		/* 
		* 磁力计 (50Hz) */
		if (qmc_task_flag != 0U)
		{
			qmc_task_flag = 0U;
			Angle_XY = QMC_Data();
			IMU_Yaw_CorrectMag(Angle_XY);   /* 磁力计校正偏航漂移 */
			IMU_Yaw = IMU_Get_Yaw();	/* 同步更新融合偏航角，供外部调用 */
		}

		/* 
		*气压计 (20Hz) */
		if (bmp_task_flag != 0U)
		{
			bmp_task_flag = 0U;
			alt = BMP_Data();
			Altitude_Update(aacz, alt, 0.05f);
		}

		/*
		*串口打印 (10Hz) */
		#if (DEBUG_PRINT_ENABLE == 1U)
			if (print_task_flag != 0U)
			{
				print_task_flag = 0U;
				/* 磁力计数据测试 */
				// usart_printf(USART1, "QMC=%.1f  IMU=%.1f  Gz=%.1f  bias=%.2f\r\n", Angle_XY, IMU_Yaw, (float)gyroz / GYRO_SENS_2000DPS, IMU_Get_GyroBias());
				/* 气压计数据测试 */
				usart_printf(USART3, "alt: %.1f aacz: %hd F: %.1f\r\n", alt, aacz, Alt_Fused);
				/* 陀螺仪数据测试 */
				// usart_printf(USART1, "Pitch=%.2f Roll=%.2f\r\n", Pitch, Roll);
				/* 3个传感器数据 */
				// usart_printf(USART1, "Pitch=%.1f Roll=%.1f IMU=%.1f alt: %.1f\r\n", Pitch, Roll, IMU_Yaw, alt); /* 无线串口 */
			}
		#endif
	}
}
