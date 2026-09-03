#include "Control_Task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "Motor.h"
#include "NRF24L01.h"
#include "MTF02P.h"

/* ────────────────────────────────────────────────────────────────
 * FreeRTOS 钩子函数（应用层，由内核回调）
 * ──────────────────────────────────────────────────────────────── */

/*
 * vApplicationStackOverflowHook — 栈溢出检测钩子（configCHECK_FOR_STACK_OVERFLOW=2）
 * 被调用时任务已严重损坏，唯一能做的：关闭所有中断 + 死循环，方便调试器检查。
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	(void)xTask;
	(void)pcTaskName;
	__asm volatile( "cpsid i" );
	for ( ;; ) { }
}

/*
 * vApplicationMallocFailedHook — 堆分配失败钩子（configUSE_MALLOC_FAILED_HOOK=1）
 * heap_4 内存不足时调用，死循环便于调试定位。
 */
void vApplicationMallocFailedHook( void )
{
	__asm volatile( "cpsid i" );
	for ( ;; ) { }
}
#include "LED.h"
#include "KEY.h"

/* ────────────────────────────────────────────────────────────────
 * RTOS 对象
 * ──────────────────────────────────────────────────────────────── */
static SemaphoreHandle_t xControlSem;      /* TIM3 → ControlTask (500Hz) */
static SemaphoreHandle_t xMpuSem;          /* EXTI → SensorTask (200Hz) */
static SemaphoreHandle_t xAttitudeMutex;   /* 姿态数据互斥锁（SensorTask 写 / ControlTask 读） */
static SemaphoreHandle_t xPrintMutex;      /* printf 互斥锁（多任务防交织） */

/* ────────────────────────────────────────────────────────────────
 * 串口打印开关：起飞前设 0 关闭所有 printf
 * ──────────────────────────────────────────────────────────────── */
#define DEBUG_PRINT_ENABLE  1U

/* ────────────────────────────────────────────────────────────────
 * ControlTask — 500Hz 姿态控制（最高优先级 6）
 *
 * TIM2 每 2ms 给一次信号量 → 500Hz 控制节拍。
 * 业务逻辑（原 TIM1 ISR 内容）：
 *   1) 读姿态共享数据（互斥锁保护）；
 *   2) 已解锁 → Pitch/Roll 串级 PID + 偏航角速度环 + 三轴混控 + DShot 输出；
 *      未解锁 → Motor_Test（电机掉电保护状态机）。
 * ──────────────────────────────────────────────────────────────── */
static void ControlTask(void *pvParameters)
{
	(void)pvParameters;

	for (;;)
	{
		/* 阻塞等 TIM3 500Hz 节拍（永不超时） */
		xSemaphoreTake(xControlSem, portMAX_DELAY);

		/* ── 读姿态共享数据（互斥锁保护，保持快照一致性）── */
		xSemaphoreTake(xAttitudeMutex, portMAX_DELAY);
		float pitch = Pitch;
		float roll  = Roll;
		short gz    = gyroz;
		xSemaphoreGive(xAttitudeMutex);

		/* PID 控制-电机混控 */
		if (Key == 1U)
		{
			PID_Pitch_Roll_Combined(pitch, roll);                    /* Pitch/Roll 串级 PID */
			PID_Yaw_Rate_Control((float)gz / GYRO_SENS_2000DPS);     /* 偏航角速度环（消除自旋） */
			Motor_Test();                                            /* 三轴混控 → DShot_Write */
		}
		else
		{
			Motor_Test(); /* 未解锁时仍走电机状态机（处理掉电缓降） */
		}
	}
}

/* ────────────────────────────────────────────────────────────────
 * SensorTask — 200Hz 读 MPU6050 DMP + 陀螺 + 加速度（优先级 5）
 *
 * MPU6050 EXTI 给信号量 → 读传感器（I2C，必须在任务里跑，不在 ISR 里跑 I2C）。
 * 读完后用互斥锁一次性提交到全局姿态变量，保证 ControlTask 读到完整快照。
 * ──────────────────────────────────────────────────────────────── */
static void SensorTask(void *pvParameters)
{
	(void)pvParameters;

	float  p, r, y;
	short  gx, gy, gz;
	short  ax, ay, az;

	for (;;)
	{
		/* 阻塞等 MPU6050 EXTI 200Hz 信号量 */
		xSemaphoreTake(xMpuSem, portMAX_DELAY);

		/* 读传感器到局部变量（I2C 事务，不在互斥锁内） */
		mpu_dmp_get_data(&p, &r, &y);
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		MPU_Get_Accelerometer(&ax, &ay, &az);

		/* 一次性提交到全局变量（互斥锁保护，临界区极短） */
		xSemaphoreTake(xAttitudeMutex, portMAX_DELAY);
		Pitch  = p;   Roll   = r;  Yaw   = y;
		gyrox  = gx;  gyroy  = gy; gyroz = gz;
		aacx   = ax;  aacy   = ay; aacz  = az;
		xSemaphoreGive(xAttitudeMutex);
	}
}

/* ────────────────────────────────────────────────────────────────
 * Mtf02pTask — MTF-02P 光流测距协议解析（优先级 4）
 *
 * 非阻塞轮询，2ms 周期消费 ISR 入队的字节（MTF02P_RxPush 由 USART ISR 调用）。
 * Micolink 帧校验通过后自动刷新 mtf02p_data（距离 mm + 光流速度 cm/s@1m + 质量状态），
 * 供后续定高定点使用。
 * ──────────────────────────────────────────────────────────────── */
static void Mtf02pTask(void *pvParameters)
{
	(void)pvParameters;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
		MTF02P_Task();
	}
}

/* ────────────────────────────────────────────────────────────────
 * RadioTask — 100Hz NRF24L01 遥控 + 遥测（优先级 3）
 * ──────────────────────────────────────────────────────────────── */
static void RadioTask(void *pvParameters)
{
	(void)pvParameters;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
		NRF24L01_RX_Data(); // 接收 + 解析遥控指令
		NRF24L01_TX_Data(); // 回传
	}
}

/* ────────────────────────────────────────────────────────────────
 * TelemetryTask — 10Hz 串口打印（优先级 2）
 *
 * printf 被 xPrintMutex 保护，允许多任务打印不交织。
 * DEBUG_PRINT_ENABLE=0 时跳过打印，只休眠。
 * ──────────────────────────────────────────────────────────────── */
static void TelemetryTask(void *pvParameters)
{
	(void)pvParameters;

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

#if (DEBUG_PRINT_ENABLE == 1U)
		xSemaphoreTake(xPrintMutex, portMAX_DELAY);
		// usart_printf(USART1, "Pitch=%.2f Roll=%.2f\r\n", Pitch, Roll); /* 姿态打印 */
		usart_printf(USART1, "gz=%.1f dps yaw_out=%.1f\r\n",
		             (double)((float)gyroz / GYRO_SENS_2000DPS),
		             (double)pid_rate_yaw.output);                    /* 偏航环调试打印 */
		// usart_printf(USART1, "dist=%lu mm flow=(%d,%d) q=%u st=%u/%u\r\n",
		//              (unsigned long)mtf02p_data.distance,
		//              mtf02p_data.flow_x, mtf02p_data.flow_y,
		//              mtf02p_data.flow_quality,
		//              mtf02p_data.tof_status, mtf02p_data.flow_status);  /* MTF02P 测试打印 */
		// usart_printf(USART1, "Key=%d speed_temp=%d R_H=%d\r\n", Key, speed_temp, R_H); /* NRF24L01测试打印 */
		xSemaphoreGive(xPrintMutex);
#endif
	}
}

/* ────────────────────────────────────────────────────────────────
 * Control_Task_RTOSInit — 创建所有 RTOS 对象
 *
 * 必须在 main() 中硬件外设（TIM2/MPU6050 EXTI）启动前调用，确保 ISR 给信号量时对象已存在。
 * ──────────────────────────────────────────────────────────────── */
void Control_Task_RTOSInit(void)
{
	/* 创建信号量（初始为 0，由 ISR 给） */
	xControlSem = xSemaphoreCreateBinary();
	xMpuSem     = xSemaphoreCreateBinary();

	/* 创建互斥锁（优先级继承，防止翻转） */
	xAttitudeMutex = xSemaphoreCreateMutex();
	xPrintMutex    = xSemaphoreCreateMutex();

	/* 创建任务（栈深度单位：字 = 4 字节） */
	xTaskCreate(ControlTask,    "Control",  512, NULL, 6, NULL);
	xTaskCreate(SensorTask,     "Sensor",   512, NULL, 5, NULL);
	xTaskCreate(Mtf02pTask,     "Mtf02p",   256, NULL, 4, NULL);
	xTaskCreate(RadioTask,      "Radio",    256, NULL, 3, NULL);
	xTaskCreate(TelemetryTask,  "Telem",    256, NULL, 2, NULL);
}

/* ────────────────────────────────────────────────────────────────
 * ISR 回调（由 main.c 注册到 Enroll/API 层）
 * ──────────────────────────────────────────────────────────────── */

/*
 * Control_Task1_Callback — TIM2 2ms ISR
 *
 * 每 2ms = 500Hz 给 xControlSem 信号量，唤醒 ControlTask。
 * TIM2 直接按 2ms 周期配置（API_TIM_Init periodMs=2），ISR 内不再分频。
 */
void Control_Task1_Callback(API_TIM_Id_t id)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (id != API_TIM1)
	{
		return;
	}

	if (xControlSem == NULL)
	{
		return;
	}

	xSemaphoreGiveFromISR(xControlSem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * ControlTask_NotifyMpuIsr — 由 MPU6050 EXTI 回调调用
 *
 * 给 xMpuSem 信号量，唤醒 SensorTask。
 * 调用上下文：MPU6050_EXTI_Callback（ISR，优先级 6，可调 FromISR）。
 */
void ControlTask_NotifyMpuIsr(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (xMpuSem != NULL)
	{
		xSemaphoreGiveFromISR(xMpuSem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/*
 * Control_Task_USART_Callback — USART 中断回调（所有 USART 共用）
 *
 * 保持 FreeRTOS 不感知（USART 优先级 4 < configMAX_SYSCALL=5），
 * 不调用任何 FreeRTOS API。仅做硬件搬运：TX 队列排空 + RX 按串口分发。
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t  rxValid;

	do
	{
		data    = 0U;
		rxValid = 0U;
		usart_irq_dispatch_by_id(id, &data, &rxValid);

		if (rxValid != 0U)
		{
			/* 
			*MTF-02P 光流测距：
			*ISR 只入队，协议解析在 Mtf02pTask 中进行 */
			if (id == MTF02P_USART_ID)
			{
				MTF02P_RxPush((uint8_t)data);
			}
		}
	} while (rxValid != 0U);
}
