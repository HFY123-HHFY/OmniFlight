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

/* 死机诊断：轮询方式把消息打到 USART1（不依赖中断/内核，中断全关也能发）。 */
static void Hang_PrintPolling(const char *msg)
{
	const char *p;

	if (msg == 0)
	{
		return;
	}
	for (p = msg; *p != '\0'; p++)
	{
		API_USART_WriteByte(API_USART1, (uint8_t)*p);
	}
}

/*
 * vApplicationStackOverflowHook — 栈溢出检测钩子（configCHECK_FOR_STACK_OVERFLOW=2）
 * 被调用时任务已严重损坏：先在串口上打印是哪个任务溢出（轮询发送，不依赖内核），
 * 再关闭所有中断 + 死循环，方便调试器检查。
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	(void)xTask;

	Hang_PrintPolling("\r\n[STACK OVERFLOW] ");
	Hang_PrintPolling(pcTaskName);
	Hang_PrintPolling("\r\n");
	__asm volatile( "cpsid i" );
	for ( ;; ) { }
}

/*
 * vApplicationMallocFailedHook — 堆分配失败钩子（configUSE_MALLOC_FAILED_HOOK=1）
 * heap_4 内存不足时调用，打印提示后死循环便于调试定位。
 */
void vApplicationMallocFailedHook( void )
{
	Hang_PrintPolling("\r\n[HEAP OUT OF MEMORY]\r\n");
	__asm volatile( "cpsid i" );
	for ( ;; ) { }
}

/*
 * HardFault_Handler — 覆盖启动文件的 weak 定义。
 * 硬件异常（野指针/未对齐/总线错误等）都会落到这里：
 * 打印提示后死循环，用调试器看 CFSR/PC/LR 定位。
 */
void HardFault_Handler(void)
{
	Hang_PrintPolling("\r\n[HARDFAULT]\r\n");
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

/* 任务句柄（栈水位监控用） */
static TaskHandle_t s_controlTaskHandle;
static TaskHandle_t s_sensorTaskHandle;
static TaskHandle_t s_mtf02pTaskHandle;
static TaskHandle_t s_radioTaskHandle;
static TaskHandle_t s_telemetryTaskHandle;
static TaskHandle_t s_ledTaskHandle;

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
 *   2) 已解锁（Key==1 手动 / Key==3 定高）→ Pitch/Roll 串级 PID + 偏航角速度环
 *      + 定高环 + 定点环 + 三轴混控 + DShot 输出；
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

		/* PID 控制-电机混控（Key==1 手动 / Key==3 定高，均为解锁态） */
		if ((Key == 1U) || (Key == 3U))
		{
			Pos_Control();                                           /* 定点环（100Hz，写 Target_Pitch/Roll，须在串级 PID 前） */
			PID_Pitch_Roll_Combined(pitch, roll);                    /* Pitch/Roll 串级 PID */
			PID_Yaw_Rate_Control((float)gz / GYRO_SENS_2000DPS);     /* 偏航角速度环（消除自旋） */
			Alt_Control();                                           /* 定高环（内部 100Hz 降采样） */
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
 * 供定高环/定点环使用。
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

	for (;;)
	{
		NRF24L01_RX_Data(); // 接收 + 解析遥控指令
		NRF24L01_TX_Data(); // 回传
		/* 收发函数可能因无线重发耗时，操作后强制让出 CPU，避免周期追赶占满任务。 */
		vTaskDelay(pdMS_TO_TICKS(10));
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
		// usart_printf(USART1, "gz=%.1f dps yaw_out=%.1f\r\n",
		//              (double)((float)gyroz / GYRO_SENS_2000DPS),
		//              (double)pid_rate_yaw.output);                    /* 偏航环调试打印 */
		// usart_printf(USART1, "d=%.2f t=%.2f v=%.2f out=%.0f RH=%d\r\n",
		//              (double)((float)mtf02p_data.distance / 1000.0f),
		//              (double)Alt_Target_M, (double)Alt_Rate_Mps,
		//              (double)Alt_Throttle_Out, Altitude_Stick_Input); /* 定高环调试打印 */
		// usart_printf(USART1, "dist=%lu mm flow=(%d,%d) q=%u st=%u/%u\r\n",
		//              (unsigned long)mtf02p_data.distance,
		//              mtf02p_data.flow_x, mtf02p_data.flow_y,
		//              mtf02p_data.flow_quality,
		//              mtf02p_data.tof_status, mtf02p_data.flow_status);  /* MTF02P 测试打印 */
		// usart_printf(USART1, "Key=%d speed_temp=%d Altitude_Stick_Input=%d\r\n", Key, speed_temp, Altitude_Stick_Input); /* NRF24L01测试打印 */
		usart_printf(USART1, "A=%u q=%u h=%.1f pe=(%.2f,%.2f) pt=(%.2f,%.2f) sp=(%.2f,%.2f) t=(%.1f,%.1f)\r\n",
		             Pos_Active,
		             mtf02p_data.flow_quality,
		             (double)((float)mtf02p_data.distance / 1000.0f),
		             (double)Pos_EstX_M, (double)Pos_EstY_M,
		             (double)Pos_TarX_M, (double)Pos_TarY_M,
		             (double)Pos_SpX_Mps, (double)Pos_SpY_Mps,
		             (double)Pos_TiltX_Deg, (double)Pos_TiltY_Deg); /* 定点环调试打印：A=生效 q=光流质量(≥40生效) h=高度(0.1~4.0m窗内) pe=位置估计(跟着手的位移走) pt=位置目标(摇杆积分,回中冻结) sp=速度指令(位置外环+前馈) t=倾角输出；调速度内环时再补 e/p/i/v */
		/* 每 1s 打印一次各任务栈剩余水位（字）：哪个任务逼近 0 就是卡死隐患 */
		// {
		// 	static uint8_t wmCount = 0U;

		// 	wmCount++;
		// 	if (wmCount >= 10U)
		// 	{
		// 		wmCount = 0U;
		// 		usart_printf(USART1, "WM Ctl=%u Sen=%u Mtf=%u Rad=%u Tel=%u Led=%u\r\n",
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_controlTaskHandle),
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_sensorTaskHandle),
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_mtf02pTaskHandle),
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_radioTaskHandle),
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_telemetryTaskHandle),
		// 		             (unsigned)uxTaskGetStackHighWaterMark(s_ledTaskHandle));
		// 	}
		// }
		xSemaphoreGive(xPrintMutex);
#endif
	}
}

/* ────────────────────────────────────────────────────────────────
 * LEDTask — LED 状态指示（最低应用优先级 1）
 *
 * 状态优先级从高到低：断链绿色闪烁、解锁 RGB 交替、锁定红色闪烁、
 * 已连接待命绿色常亮。每次切换状态先关闭全部 LED，避免旧状态残留。
 * ──────────────────────────────────────────────────────────────── */
static void LEDTask(void *pvParameters)
{
	uint8_t tickCount = 0U;
	uint8_t effectStep = 0U;
	uint8_t lastMode = 0xFFU;
	uint8_t mode;

	(void)pvParameters;

	for (;;)
	{
		if (NRF24L01_Linked == 0U)
		{
			mode = 0U; /* 断链：绿色闪烁 */
		}
		else if ((Key == 1U) || (Key == 3U))
		{
			mode = 1U; /* 解锁（1=手动 / 3=定高）：RGB 交替 */
		}
		else if (Key == 2U)
		{
			mode = 2U; /* 锁定：红色闪烁 */
		}
		else
		{
			mode = 3U; /* 已连接待命：绿色常亮 */
		}

		if (mode != lastMode)
		{
			LED_Control(LED1, LED_LOW);
			LED_Control(LED2, LED_LOW);
			LED_Control(LED3, LED_LOW);
			tickCount = 0U;
			effectStep = 0U;
			lastMode = mode;

			if (mode == 3U)
			{
				LED_Control(LED1, LED_HIGH);
			}
		}
		else if ((mode == 0U) || (mode == 1U) || (mode == 2U))
		{
			tickCount++;
			if (tickCount >= 5U) /* LEDTask 100ms 一次，5 次为 500ms */
			{
				tickCount = 0U;

				if (mode == 0U)
				{
					LED_Control(LED1, (effectStep == 0U) ? LED_HIGH : LED_LOW);
					effectStep = (effectStep == 0U) ? 1U : 0U;
				}
				else if (mode == 2U)
				{
					LED_Control(LED2, (effectStep == 0U) ? LED_HIGH : LED_LOW);
					effectStep = (effectStep == 0U) ? 1U : 0U;
				}
				else
				{
					LED_Control(LED1, LED_LOW);
					LED_Control(LED2, LED_LOW);
					LED_Control(LED3, LED_LOW);
					LED_Control((LED_Id_t)(LED1 + effectStep), LED_HIGH);
					effectStep = (effectStep + 1U) % 3U;
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));
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

	/* 创建任务（栈深度单位：字 = 4 字节）。
	 * 注意：溢出 = 全局死机（钩子关中断），打印类/协议类任务给足余量，
	 * 并用 1s 一次的水位监控（WM 行）观察真实使用量再精调。 */
	xTaskCreate(ControlTask,    "Control",  512, NULL, 6, &s_controlTaskHandle);
	xTaskCreate(SensorTask,     "Sensor",   512, NULL, 5, &s_sensorTaskHandle);
	xTaskCreate(Mtf02pTask,     "Mtf02p",   320, NULL, 4, &s_mtf02pTaskHandle);
	xTaskCreate(RadioTask,      "Radio",    384, NULL, 3, &s_radioTaskHandle);
	xTaskCreate(TelemetryTask,  "Telem",    384, NULL, 2, &s_telemetryTaskHandle);
	xTaskCreate(LEDTask,        "LED",      256, NULL, 1, &s_ledTaskHandle);
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
