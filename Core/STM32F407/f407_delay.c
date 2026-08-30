#include "delay.h"
#include "stm32f4xx.h"

/* 由 CMSIS 系统文件维护：表示当前 HCLK 频率（单位 Hz）。 */
extern uint32_t SystemCoreClock;

/*
 * F407 延时实现 —— 基于 DWT 周期计数器（CYCCNT）。
 *
 * 为什么不用 SysTick：
 *   FreeRTOS 独占 SysTick 作为系统节拍（xPortSysTickHandler）。
 *   若 Delay 再操作 SysTick->LOAD/CTRL 会破坏 RTOS 节拍，故改用 DWT。
 *
 * DWT->CYCCNT 是 Cortex-M4 内核的 32bit 自由运行周期计数器，与 SysTick 完全无关，
 * 在调度器启动前后均可安全使用。无符号减法天然处理计数器回绕。
 */

/* 惰性初始化：首次调用延时时使能 DWT 的 CYCCNT。 */
static void F407_DwtInit(void)
{
	static uint8_t done = 0U;

	if (done != 0U)
	{
		return;
	}
	done = 1U;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能 DWT/ITM trace */
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             /* 启动周期计数器 */
}

/* 微秒级忙等延时。 */
void Delay_us(uint32_t us)
{
	uint32_t start;
	uint32_t ticks;

	if (us == 0U)
	{
		return;
	}

	F407_DwtInit();
	start = DWT->CYCCNT;
	ticks = us * (SystemCoreClock / 1000000U);

	/* (now - start) 无符号差值，自动跨越 CYCCNT 回绕点 */
	while ((uint32_t)(DWT->CYCCNT - start) < ticks)
	{
	}
}

/* 毫秒级忙等延时（分段，留出计数器余量）。 */
void Delay_ms(uint32_t ms)
{
	while (ms > 0U)
	{
		uint32_t chunk = (ms > 1000U) ? 1000U : ms;
		Delay_us(chunk * 1000U);
		ms -= chunk;
	}
}

void Delay_s(uint32_t s)
{
	while (s > 0U)
	{
		Delay_ms(1000U);
		--s;
	}
}

/*
 * 毫秒级单调时钟：自首次调用起累计，供上层模块做非阻塞计时（如传感器校准时长）。
 * 无符号差值天然处理 CYCCNT 回绕，但两次调用间隔需小于回绕周期（168MHz 下约 25.5s）。
 */
uint32_t Delay_GetMs(void)
{
	static uint32_t last = 0U;
	static uint32_t acc  = 0U;

	F407_DwtInit();
	acc += (uint32_t)(DWT->CYCCNT - last) / (SystemCoreClock / 1000U);
	last = DWT->CYCCNT;

	return acc;
}
