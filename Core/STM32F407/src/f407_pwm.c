#include "f407_pwm.h"

/* F407 PWM 说明：
 * - 使用定时器输出比较 PWM 模式 1
 * - PWM 输出引脚配置为复用功能模式
 * - 这里只负责底层寄存器配置，不负责板级映射注册
 */

/* F407 通用定时器寄存器视图（TIM2~TIM5/TIM9）。 */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
} F407_PWM_GenRegs_t;

/* F407 高级定时器寄存器视图（TIM1）。 */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t RCR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t BDTR;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
} F407_PWM_AdvRegs_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
} F407_PWM_RccRegs_t;

typedef struct
{
	void *regs;
	uint8_t advanced;
	uint32_t clockHz;
	uint32_t rccBit;
	uint8_t apb2;
} F407_PWM_Map_t;

#define F407_PWM_RCC_BASE          (0x40023800UL)
#define F407_PWM_TIM1_BASE         (0x40010000UL)
#define F407_PWM_TIM2_BASE         (0x40000000UL)
#define F407_PWM_TIM3_BASE         (0x40000400UL)
#define F407_PWM_TIM4_BASE         (0x40000800UL)
#define F407_PWM_TIM5_BASE         (0x40000C00UL)
#define F407_PWM_TIM9_BASE         (0x40014000UL)

#define F407_PWM_RCC               ((F407_PWM_RccRegs_t *)F407_PWM_RCC_BASE)

/* F407 系统下：TIM1/TIM9 走 APB2，TIM2~TIM5 走 APB1。 */
#define F407_PWM_TIM1_CLOCK_HZ      (168000000UL)
#define F407_PWM_TIMx_CLOCK_HZ      (84000000UL)
#define F407_PWM_TIM9_CLOCK_HZ      (168000000UL)

static F407_PWM_Map_t F407_PWM_GetMap(uint8_t timId)
{
	F407_PWM_Map_t map;

	map.regs = 0;
	map.advanced = 0U;
	map.clockHz = F407_PWM_TIMx_CLOCK_HZ;
	map.rccBit = 0U;
	map.apb2 = 0U;

	switch (timId)
	{
	case 1U:
		map.regs = (F407_PWM_AdvRegs_t *)F407_PWM_TIM1_BASE;
		map.advanced = 1U;
		map.clockHz = F407_PWM_TIM1_CLOCK_HZ;
		map.rccBit = 0U;
		map.apb2 = 1U;
		break;
	case 2U:
		map.regs = (F407_PWM_GenRegs_t *)F407_PWM_TIM2_BASE;
		map.rccBit = 0U;
		break;
	case 3U:
		map.regs = (F407_PWM_GenRegs_t *)F407_PWM_TIM3_BASE;
		map.rccBit = 1U;
		break;
	case 4U:
		map.regs = (F407_PWM_GenRegs_t *)F407_PWM_TIM4_BASE;
		map.rccBit = 2U;
		break;
	case 5U:
		map.regs = (F407_PWM_GenRegs_t *)F407_PWM_TIM5_BASE;
		map.rccBit = 3U;
		break;
	case 9U:
		map.regs = (F407_PWM_GenRegs_t *)F407_PWM_TIM9_BASE;
		map.clockHz = F407_PWM_TIM9_CLOCK_HZ;
		map.rccBit = 16U;
		map.apb2 = 1U;
		break;
	default:
		break;
	}

	return map;
}

static uint8_t F407_PWM_GetAfNum(uint8_t timId)
{
	if (timId == 1U)
	{
		return 1U;
	}

	if (timId == 9U)
	{
		return 3U;
	}

	return 2U;
}

/* 根据定时器选择 AF，并把指定 GPIO 引脚配置成 PWM 复用功能。 */
void F407_PWM_ConfigPin(void *port, uint16_t pin, uint8_t timId)
{
	F407_GPIO_Regs_t *gpioPort;
	uint32_t pinIndex;
	uint32_t shift;
	uint8_t af;

	if ((port == 0) || (pin == 0U))
	{
		return;
	}

	gpioPort = (F407_GPIO_Regs_t *)port;
	af = F407_PWM_GetAfNum(timId);
	F407_GPIO_EnablePortClock(port);
	pinIndex = F407_GPIO_PinIndex(pin);
	if (pinIndex > 15U)
	{
		return;
	}

	/* PWM 输出在 F407 上配置为复用功能模式。 */
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
 * 配置指定通道为 PWM 模式 1，并更新 CCR。
 * 对 TIM1（高级定时器）与 TIM2~TIM5/TIM9（通用定时器）分别处理。
 */
static void F407_PWM_SetChannel(void *regs, uint8_t advanced, uint8_t channel, uint32_t compare)
{
	if (advanced != 0U)
	{
		F407_PWM_AdvRegs_t *tim = (F407_PWM_AdvRegs_t *)regs;

		switch (channel)
		{
		case 1U:
			tim->CCMR1 &= ~((7UL << 4) | (1UL << 3));
			tim->CCMR1 |= ((6UL << 4) | (1UL << 3));
			tim->CCER |= (1UL << 0);
			tim->CCR1 = compare;
			break;
		case 2U:
			tim->CCMR1 &= ~((7UL << 12) | (1UL << 11));
			tim->CCMR1 |= ((6UL << 12) | (1UL << 11));
			tim->CCER |= (1UL << 4);
			tim->CCR2 = compare;
			break;
		case 3U:
			tim->CCMR2 &= ~((7UL << 4) | (1UL << 3));
			tim->CCMR2 |= ((6UL << 4) | (1UL << 3));
			tim->CCER |= (1UL << 8);
			tim->CCR3 = compare;
			break;
		case 4U:
			tim->CCMR2 &= ~((7UL << 12) | (1UL << 11));
			tim->CCMR2 |= ((6UL << 12) | (1UL << 11));
			tim->CCER |= (1UL << 12);
			tim->CCR4 = compare;
			break;
		default:
			break;
		}
	}
	else
	{
		F407_PWM_GenRegs_t *tim = (F407_PWM_GenRegs_t *)regs;

		switch (channel)
		{
		case 1U:
			tim->CCMR1 &= ~((7UL << 4) | (1UL << 3));
			tim->CCMR1 |= ((6UL << 4) | (1UL << 3));
			tim->CCER |= (1UL << 0);
			tim->CCR1 = compare;
			break;
		case 2U:
			tim->CCMR1 &= ~((7UL << 12) | (1UL << 11));
			tim->CCMR1 |= ((6UL << 12) | (1UL << 11));
			tim->CCER |= (1UL << 4);
			tim->CCR2 = compare;
			break;
		case 3U:
			tim->CCMR2 &= ~((7UL << 4) | (1UL << 3));
			tim->CCMR2 |= ((6UL << 4) | (1UL << 3));
			tim->CCER |= (1UL << 8);
			tim->CCR3 = compare;
			break;
		case 4U:
			tim->CCMR2 &= ~((7UL << 12) | (1UL << 11));
			tim->CCMR2 |= ((6UL << 12) | (1UL << 11));
			tim->CCER |= (1UL << 12);
			tim->CCR4 = compare;
			break;
		default:
			break;
		}
	}
}

/*
 * 初始化定时器基础参数：
 * - 设置 PSC/ARR
 * - 触发更新事件装载预装载寄存器
 * - 启动计数器
 */
void F407_PWM_InitTimer(uint8_t timId, uint16_t arr, uint16_t psc)
{
	F407_PWM_Map_t map;

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	{
		F407_PWM_GenRegs_t *common = (F407_PWM_GenRegs_t *)map.regs;

		if (map.apb2 != 0U)
		{
			F407_PWM_RCC->APB2ENR |= (1UL << map.rccBit);
		}
		else
		{
			F407_PWM_RCC->APB1ENR |= (1UL << map.rccBit);
		}

		common->CR1 &= ~(1UL << 0);
		common->CR1 |= (1UL << 7); /* ARPE=1。 */
		common->CR2 = 0U;
		common->SMCR = 0U;
		common->PSC = (uint32_t)psc;
		common->ARR = (uint32_t)arr;
		common->EGR = 1U;
		common->SR = 0U;

		if (map.advanced != 0U)
		{
			((F407_PWM_AdvRegs_t *)map.regs)->BDTR |= (1UL << 15); /* MOE=1。 */
		}

		common->CR1 |= (1UL << 0);
	}
}

/* 设置指定定时器通道的比较值（CCR）。 */
void F407_PWM_SetCCR(uint8_t timId, uint8_t channel, uint16_t ccr)
{
	F407_PWM_Map_t map;

	if ((channel < 1U) || (channel > 4U))
	{
		return;
	}

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	F407_PWM_SetChannel(map.regs, map.advanced, channel, ccr);
}

/*
 * 获取定时器外设基地址。
 * 用于 DMA 配置或高级协议层直接寄存器访问。
 * 返回值: 定时器基地址，无效 ID 返回 0。
 */
uint32_t F407_PWM_GetTimerBase(uint8_t timId)
{
	F407_PWM_Map_t map;

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return 0U;
	}

	return (uint32_t)map.regs;
}

/*
 * 配置高级定时器（TIM1）的 DMA burst。
 * - dba: 目标寄存器偏移 / 4（如 CCR1 偏移 0x34 → dba=0x0D）
 * - burstLen: burst 传输次数编码（0=1次, 1=2次, 2=3次, 3=4次）
 * 仅对 TIM1 有效。
 */
void F407_PWM_ConfigDMABurst(uint8_t timId, uint8_t dba, uint8_t burstLen)
{
	F407_PWM_Map_t map;

	/* 仅高级定时器（TIM1）支持 DCR */
	if (timId != 1U)
	{
		return;
	}

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	{
		F407_PWM_AdvRegs_t *tim = (F407_PWM_AdvRegs_t *)map.regs;

		/* DCR: DBA[4:0] + DBL[12:8] */
		tim->DCR = ((uint32_t)dba & 0x1FU)
		         | (((uint32_t)burstLen & 0x1FU) << 8U);
	}
}

/*
 * 使能定时器 Update 事件的 DMA 请求。
 * 使能后，每次定时器溢出将触发 DMA 传输。
 */
void F407_PWM_EnableUpdateDMA(uint8_t timId)
{
	F407_PWM_Map_t map;

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	{
		F407_PWM_GenRegs_t *common = (F407_PWM_GenRegs_t *)map.regs;

		/* DIER 寄存器布局在通用/高级定时器中 UDE 位均为 bit8 */
		common->DIER |= (1UL << 8U);
	}
}

/*
 * 触发定时器更新事件（EGR.UG）。
 * 将 ARR/PSC/CCR 预装载值同步到影子寄存器。
 */
void F407_PWM_ForceUpdate(uint8_t timId)
{
	F407_PWM_Map_t map;

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	{
		F407_PWM_GenRegs_t *common = (F407_PWM_GenRegs_t *)map.regs;

		common->EGR |= 1U;
	}
}

/*
 * 直接设置定时器 ARR 值。
 * 仅写 ARR 寄存器，不改 PSC/CR1/CR2 等，适合运行时动态调频。
 * ARPE=1 时写入的影子寄存器在下次更新事件生效。
 */
void F407_PWM_SetARR(uint8_t timId, uint16_t arr)
{
	F407_PWM_Map_t map;

	map = F407_PWM_GetMap(timId);
	if (map.regs == 0)
	{
		return;
	}

	{
		F407_PWM_GenRegs_t *common = (F407_PWM_GenRegs_t *)map.regs;

		common->ARR = (uint32_t)arr;
	}
}
