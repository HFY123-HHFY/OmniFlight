#include "usart.h"
#include "gpio.h"

static const API_USART_Config_t *s_usartTable;
static uint8_t s_usartCount;
static API_USART_IrqHandler_t s_usartIrqHandlers[API_USART3 + 1U];

#ifndef API_USART_CR1_TXEIE
#define API_USART_CR1_TXEIE (1UL << 7)
#endif

/* USART1/2/3 对应的复用功能号。 */
static uint8_t API_USART_GetAfNum(API_USART_Id_t id)
{
	/* STM32F407 所有 USART1/2/3 均使用 AF7 */
	(void)id;
	return 7U;
}

/* 配置 F407 的 TX/RX 引脚为复用模式。 */
static void API_USART_ConfigAfPin(void *port, uint16_t pin, uint8_t af)
{
	F407_GPIO_Regs_t *gpioPort;
	uint32_t pinIndex;
	uint32_t shift;

	if ((port == 0) || (pin == 0U))
	{
		return;
	}

	gpioPort = (F407_GPIO_Regs_t *)port;
	F407_GPIO_EnablePortClock(port);
	pinIndex = F407_GPIO_PinIndex(pin);
	if (pinIndex > 15U)
	{
		return;
	}

	shift = pinIndex * 2U;
	gpioPort->MODER &= ~(0x3UL << shift);
	gpioPort->MODER |= (0x2UL << shift);
	gpioPort->OTYPER &= ~(1UL << pinIndex);
	gpioPort->OSPEEDR &= ~(0x3UL << shift);
	gpioPort->OSPEEDR |= (0x2UL << shift);
	gpioPort->PUPDR &= ~(0x3UL << shift);

	if (pinIndex < 8U)
	{
		shift = pinIndex * 4U;
		gpioPort->AFRL &= ~(0xFUL << shift);
		gpioPort->AFRL |= ((uint32_t)af << shift);
	}
	else
	{
		shift = (pinIndex - 8U) * 4U;
		gpioPort->AFRH &= ~(0xFUL << shift);
		gpioPort->AFRH |= ((uint32_t)af << shift);
	}
}

/*
 * 串口底层初始化：
 * 这层只负责注册、引脚复用和调用 Core 初始化，不处理接收中断包装。
 */
static void API_USART_CoreInit(uint8_t coreId, uint32_t baudRate)
{
	F407_USART_Init(coreId, baudRate);
}

static void API_USART_CoreWriteByte(uint8_t coreId, uint8_t data)
{
	F407_USART_WriteByte(coreId, data);
}

/* 在注册表中查找指定串口。 */
static const API_USART_Config_t *API_USART_FindConfig(API_USART_Id_t id)
{
	uint8_t index;

	if ((s_usartTable == 0) || (s_usartCount == 0U))
	{
		return 0;
	}

	for (index = 0U; index < s_usartCount; ++index)
	{
		if (s_usartTable[index].id == id)
		{
			return &s_usartTable[index];
		}
	}

	return 0;
}

/* 在注册表中按 Core USART 索引查找配置。 */
static const API_USART_Config_t *API_USART_FindConfigByCoreId(uint8_t coreId)
{
	uint8_t index;

	if ((s_usartTable == 0) || (s_usartCount == 0U))
	{
		return 0;
	}

	for (index = 0U; index < s_usartCount; ++index)
	{
		if (s_usartTable[index].coreId == coreId)
		{
			return &s_usartTable[index];
		}
	}

	return 0;
}

/* 注册板级串口资源映射表。 */
void API_USART_Register(const API_USART_Config_t *configTable, uint8_t count)
{
	s_usartTable = configTable;
	s_usartCount = count;
}

/*
 * Weak 钩子：当 USART 中断回调被注册时调用。
 * My_Usart 模块可覆盖此函数以自动使能异步 TX 模式。
 * 不覆盖则无副作用（异步 TX 不会被使能，printf 走阻塞模式）。
 */
__attribute__((weak)) void API_USART_OnIrqHandlerRegistered(API_USART_Id_t id)
{
	(void)id;
}

void API_USART_RegisterIrqHandler(API_USART_Id_t id, API_USART_IrqHandler_t handler)
{
	if ((id < API_USART1) || (id > API_USART3))
	{
		return;
	}

	s_usartIrqHandlers[id] = handler;

	if (handler != 0)
	{
		API_USART_OnIrqHandlerRegistered(id);
	}
}

/* 串口初始化入口：id 选串口，baudRate 配置波特率。 */
void API_USART_Init(API_USART_Id_t id, uint32_t baudRate)
{
	const API_USART_Config_t *config;

	config = API_USART_FindConfig(id);
	if (config == 0)
	{
		return;
	}

	if ((config->txPort != 0) && (config->txPin != 0U))
	{
		API_USART_ConfigAfPin(config->txPort, config->txPin, API_USART_GetAfNum(id));
	}

	if ((config->rxPort != 0) && (config->rxPin != 0U))
	{
		API_USART_ConfigAfPin(config->rxPort, config->rxPin, API_USART_GetAfNum(id));
	}

	if (baudRate == 0U)
	{
		return;
	}

	API_USART_CoreInit(config->coreId, baudRate);
}

/* 串口发送 1 字节。 */
void API_USART_WriteByte(API_USART_Id_t id, uint8_t data)
{
	const API_USART_Config_t *config;

	config = API_USART_FindConfig(id);
	if (config == 0)
	{
		return;
	}

	API_USART_CoreWriteByte(config->coreId, data);
}

void API_USART_HandleIrqByCoreId(uint8_t coreId)
{
	const API_USART_Config_t *config;
	API_USART_IrqHandler_t handler;

	config = API_USART_FindConfigByCoreId(coreId);
	if (config == 0)
	{
		return;
	}

	if ((config->id < API_USART1) || (config->id > API_USART3))
	{
		return;
	}

	handler = s_usartIrqHandlers[config->id];
	if (handler == 0)
	{
		return;
	}

	handler(config->id);
}

void USART1_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_USART1);
}

void USART2_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_USART2);
}

void USART3_IRQHandler(void)
{
	API_USART_HandleIrqByCoreId(API_USART_CORE_USART3);
}
