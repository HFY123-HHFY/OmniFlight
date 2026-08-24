#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * FreeRTOSConfig.h — OmniFlight (STM32F407, Cortex-M4F, 168MHz)
 *
 * 当前状态：内核已接入编译，但【调度器未启动、未接管 SysTick/PendSV/SVC】。
 *           固件行为与裸机完全一致，FreeRTOS 代码经 --gc-sections 在链接期被裁剪。
 *
 * 正式的 RTOS 移植（创建任务、对接系统节拍与异常向量、重构 Control_Task）
 * 在后续移植步骤进行，届时本文件的中断优先级等配置会一并定稿。
 */

/* ---------------- 时钟与节拍 ---------------- */
#define configCPU_CLOCK_HZ              (168000000UL)            /* F407 168MHz */
#define configTICK_RATE_HZ              ((TickType_t)1000)       /* 1kHz = 1ms 节拍 */
#define configUSE_16_BIT_TICKS          0                        /* 32 位节拍计数 */

/* ---------------- 调度器与任务 ---------------- */
#define configUSE_PREEMPTION            1                        /* 抢占式调度 */
#define configMAX_PRIORITIES            (7)
#define configMINIMAL_STACK_SIZE        ((uint16_t)128)          /* 单位:字, 128*4=512 字节 */
#define configMAX_TASK_NAME_LEN         (16)
#define configIDLE_SHOULD_YIELD         1

/* ---------------- 内存（heap_4：支持释放与合并） ---------------- */
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION  0
#define configTOTAL_HEAP_SIZE           ((size_t)(20 * 1024))    /* 20KB，移植时可再调 */

/* ---------------- 钩子（移植初期全部关闭，避免必须实现钩子函数） ---------------- */
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configUSE_MALLOC_FAILED_HOOK    0
#define configCHECK_FOR_STACK_OVERFLOW  0

/* ---------------- 可选内核功能 ---------------- */
#define configUSE_MUTEXES               1
#define configUSE_COUNTING_SEMAPHORES   1
#define configUSE_TASK_NOTIFICATIONS    1
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    (configMINIMAL_STACK_SIZE * 2)
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configUSE_TRACE_FACILITY        0
#define configUSE_CO_ROUTINES           0

/* ---------------- Cortex-M4F 中断优先级（NVIC 4 bit，逻辑优先级 0~15） ---------------- */
/*
 * 规则：任何要调用 FreeRTOS `...FromISR()` API 的中断，其逻辑优先级必须
 *       数值 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（即更不紧急）。
 *       高于它的中断（数值更小）禁止调用任何内核 API。
 *
 * 注意：本工程现有中断优先级（SysTick=0 / TIM1=1 / MPU6050=2 / TIM2=3 / USART=4）
 *       目前都高于下面的阈值，移植时需统一重排——这属于后续正式移植的工作。
 */
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY      << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 断言：配置错误时关中断并停住，便于调试定位 */
#define configASSERT(x)  do { if ((x) == 0) { __asm volatile("cpsid i" ::: "memory"); for (;;) {} } } while (0)

#endif /* FREERTOS_CONFIG_H */
