#ifndef __FREERTOS_CONFIG_H
#define __FREERTOS_CONFIG_H

/*
 * FreeRTOSConfig.h — OmniFlight 四轴飞控 (STM32F407VET6, Cortex-M4F, 168MHz)
 *
 * 关键点：
 *  - SysTick 由 FreeRTOS 独占作为系统节拍 (1ms)，Delay 已改用 DWT 计数器解耦。
 *  - 通过 #define 把 vPortSVCHandler/xPortPendSVHandler/xPortSysTickHandler
 *    映射到启动文件里的 SVC/PendSV/SysTick_Handler（覆盖 weak 别名，不改启动文件）。
 *  - 中断优先级分组：4bit (0~15)。调用 FromISR API 的中断必须 >= MAX_SYSCALL(5)。
 */

/* ===================== 时钟与节拍 ===================== */
#define configCPU_CLOCK_HZ              ( ( unsigned long ) 168000000 )
#define configTICK_RATE_HZ              ( ( TickType_t ) 1000 )   /* 1ms 系统节拍 */

/* ===================== 调度与优先级 ===================== */
#define configUSE_PREEMPTION            1                          /* 抢占式调度 */
#define configUSE_TIME_SLICING          1                          /* 同优先级时间片轮转 */
#define configMAX_PRIORITIES            7                          /* 任务优先级 0~6 */
#define configUSE_16_BIT_TICKS          0                          /* 32bit 节拍计数 */

/* ===================== 栈与堆 ===================== */
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 128 ) /* 空闲任务栈(字) */
#define configMAX_TASK_NAME_LEN         16
#define configTOTAL_HEAP_SIZE           ( ( size_t ) ( 40 * 1024 ) ) /* heap_4 40KB */

/* ===================== 功能开关 ===================== */
#define configUSE_MUTEXES               1   /* 互斥锁（含优先级继承） */
#define configUSE_COUNTING_SEMAPHORES   1
#define configUSE_TIMERS                1   /* 软件定时器（定时器服务任务） */
#define configTIMER_TASK_PRIORITY       2
#define configTIMER_QUEUE_LENGTH        8
#define configTIMER_TASK_STACK_DEPTH    ( configMINIMAL_STACK_SIZE * 2 )
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configUSE_MALLOC_FAILED_HOOK    1
#define configCHECK_FOR_STACK_OVERFLOW  2   /* 运行期栈溢出检测（调试期打开） */
#define configUSE_TRACE_FACILITY        0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* ===================== API 按需包含（FreeRTOS V11：默认关闭，必须显式启用）===================== */
#define INCLUDE_vTaskDelay              1   /* vTaskDelay / xTaskDelayUntil */
#define INCLUDE_xTaskDelayUntil         1   /* FreeRTOS V11 新函数名 */
#define INCLUDE_vTaskSuspend            1   /* vTaskSuspend（调试用） */

/* ===================== 中断优先级（Cortex-M4：4bit，0 最高 15 最低）=====================
 * configMAX_SYSCALL_INTERRUPT_PRIORITY = 5：
 *   优先级 5~15 的中断 → FreeRTOS "感知"，会被内核临界区短暂屏蔽，允许调用 ...FromISR()。
 *   优先级 0~4  的中断 → FreeRTOS "不感知"，永不被屏蔽，但严禁调用任何 FreeRTOS API。
 * 本工程分配（见 SYSTEM/IrqPriority.h）：
 *   USART=4(不感知,异步TX零风险)  TIM3控制节拍=5(给信号量)  MPU6050 EXTI=6(给信号量)
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << 4 )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << 4 )

/* ===================== 中断向量重映射 =====================
 * 启动文件 startup_stm32f407vetx_gcc.c 中 SVC/PendSV/SysTick 是 weak 别名。
 * 这里把 FreeRTOS 的移植层处理函数名映射过去，链接时覆盖 weak 定义。
 */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* ===================== 断言（死循环便于调试定位） ===================== */
#define configASSERT( x )   if( ( x ) == 0 ) { for( ;; ) { } }

#endif /* __FREERTOS_CONFIG_H */
