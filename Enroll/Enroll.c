#include "Enroll_Internal.h"
#include "IrqPriority.h"
#include "i2c_hal.h"               /* g_i2c_hal_ops */
#include "BusRate.h"               /* I2C_MODE */

/*
 * 软件/硬件 I2C 操作表（编译时二选一，宏 I2C_MODE 定义在 BusRate.h）。
 * g_i2c_hal_ops_soft: API/API_I2C/i2c_soft.c (调用 soft_i2c_hal GPIO 原语)
 * g_i2c_hal_ops_hw:   Core/STM32F407/f407_hw_i2c.c (操作片上 I2C1 寄存器)
 */
extern I2C_HAL_Ops g_i2c_hal_ops_soft;
extern I2C_HAL_Ops g_i2c_hal_ops_hw;

/*
 * Enroll.c 职责：
 * 1) 把板级映射展开成各外设配置表；
 * 2) 调用 API/BSP 的 Register/Init 完成资源登记；
 * 3) 仅做注册与门面转发，不实现具体外设控制逻辑。
 */

/*************************** API配置层 ********************************/
/*******************************PWM***********************************/
/* PWM 配置表：把 HW_PWM_MAP 展开成 API_PWM_Config_t。 */
#define ENROLL_PWM_ITEM(timId, channel, coreTimId, coreChannel, port, pin) \
	{ timId, channel, coreTimId, coreChannel, port, pin },

static const API_PWM_Config_t s_pwmTable[] =
{
	HW_PWM_MAP(ENROLL_PWM_ITEM)
};
#undef ENROLL_PWM_ITEM

/*******************************TIM***********************************/
/* TIM 配置表：把 HW_TIM_MAP 展开成 API_TIM_Config_t。 */
#define ENROLL_TIM_ITEM(id, coreId) \
	{ id, coreId },

static const API_TIM_Config_t s_timTable[] =
{
	HW_TIM_MAP(ENROLL_TIM_ITEM)
};
#undef ENROLL_TIM_ITEM

/*******************************USART***********************************/
/* USART 配置表：把 HW_USART_MAP 展开成 API_USART_Config_t。 */
#define ENROLL_USART_ITEM(id, coreId, txPort, txPin, rxPort, rxPin) \
	{ id, coreId, txPort, txPin, rxPort, rxPin },

static const API_USART_Config_t s_usartTable[] =
{
	HW_USART_MAP(ENROLL_USART_ITEM)
};
#undef ENROLL_USART_ITEM

/*************************** I2C/SPI协议配置层 ************************/
/*******************************I2C***********************************/
/* I2C 配置表：把 HW_I2C_MAP 展开成 API_I2C_Config_t。 */
#define ENROLL_I2C_ITEM(id, sclPort, sclPin, sdaPort, sdaPin, sclIomux, sdaIomux) \
	{ id, sclPort, sclPin, sclIomux, sdaPort, sdaPin, sdaIomux },

static const API_I2C_Config_t s_i2cTable[] =
{
	HW_I2C_MAP(ENROLL_I2C_ITEM)
};
#undef ENROLL_I2C_ITEM

/*******************************SPI***********************************/
/* SPI 配置表：把 HW_SPI_MAP 展开成 API_SPI_Config_t。 */
#define ENROLL_SPI_ITEM(id, csPort, csPin, sckPort, sckPin, mosiPort, mosiPin, misoPort, misoPin, csIomux, sckIomux, mosiIomux, misoIomux) \
	{ id, csPort, csPin, csIomux, sckPort, sckPin, sckIomux, mosiPort, mosiPin, mosiIomux, misoPort, misoPin, misoIomux },

static const API_SPI_Config_t s_spiTable[] =
{
	HW_SPI_MAP(ENROLL_SPI_ITEM)
};
#undef ENROLL_SPI_ITEM

/****************************** BSP配置层 *****************************/
/*******************************LED***********************************/
/* LED 配置表：把 HW_LED_MAP 展开成 LED_Config_t。 */
#define ENROLL_LED_ITEM(id, port, pin) \
	{ id, port, pin, ENROLL_GPIO_INIT_FN, ENROLL_GPIO_WRITE_FN },

/* 当前板子的 LED 注册表。 */
static const LED_Config_t s_ledTable[] =
{
	HW_LED_MAP(ENROLL_LED_ITEM)
};
#undef ENROLL_LED_ITEM

/* MPU6050 EXTI 表：登记外部中断输入引脚。 */
static const API_EXTI_Config_t s_mpuExtiTable[] =
{
	{ 0U, HW_MPU6050_INT_PORT, HW_MPU6050_INT_PIN }
};

/****************************** API资源注册层 ************************/
/* PWM 注册：登记板级 PWM 资源表。 */
void Enroll_PWM_Register(void)
{
	API_PWM_Register(s_pwmTable, HW_PWM_COUNT);
}

/* TIM 注册：登记板级 TIM 资源表。 */
void Enroll_TIM_Register(void)
{
	API_TIM_Register(s_timTable, HW_TIM_COUNT);
}

/* TIM 中断回调注册：遍历所有板级定时器。 */
void Enroll_TIM_RegisterIrqHandler(API_TIM_IrqHandler_t handler)
{
	uint8_t i;

	for (i = 0U; i < HW_TIM_COUNT; ++i)
	{
		API_TIM_RegisterIrqHandler(s_timTable[i].id, handler);
	}
}

/* USART 注册：登记板级 USART 资源表。 */
void Enroll_USART_Register(void)
{
	API_USART_Register(s_usartTable, HW_USART_COUNT);
}

/* USART 中断回调注册：遍历所有板级 USART。 */
void Enroll_USART_RegisterIrqHandler(API_USART_IrqHandler_t handler)
{
	uint8_t i;

	for (i = 0U; i < HW_USART_COUNT; ++i)
	{
		API_USART_RegisterIrqHandler(s_usartTable[i].id, handler);
	}
}

/***************** I2C/SPI协议资源注册层 *********************/
/* I2C 注册：登记资源表 + 选择软件/硬件 I2C 操作表 */
void Enroll_I2C_Register(void)
{
	API_I2C_Register(s_i2cTable, HW_I2C_COUNT);

	/* 根据 BusRate.h 中的 I2C_MODE 宏选择底层实现 */
#if I2C_MODE == I2C_MODE_HARD
	g_i2c_hal_ops = &g_i2c_hal_ops_hw;
#else
	g_i2c_hal_ops = &g_i2c_hal_ops_soft;
#endif
}

/* SPI 注册：把板级 SPI 资源表登记到 API_SPI 模块。 */
void Enroll_SPI_Register(void)
{
	API_SPI_Register(s_spiTable, HW_SPI_COUNT);
}

/***************** BSP层资源注册层 *********************/
/* LED 注册：登记板级 LED 资源表。 */
void Enroll_LED_Register(void)
{
	LED_Register(s_ledTable, HW_LED_COUNT);
}

/*
 * MPU6050 注册：
 * 1) 登记 EXTI 资源表；
 * 2) 绑定 MPU6050 中断回调；
 * 3) 配置触发沿与优先级。
 */
void Enroll_MPU6050_Register(void)
{
	API_EXTI_Register(s_mpuExtiTable, 1U);
	/* 同一 id 可继续追加其他回调。 */
	API_EXTI_AddIrqHandler(s_mpuExtiTable[0].id, (API_EXTI_IrqHandler_t)MPU6050_EXTI_Callback, NULL);
	/* API_EXTI_AddIrqHandler(s_mpuExtiTable[0].id, Other_EXTI_Callback, userPtr); */
	API_EXTI_Init(s_mpuExtiTable[0].id, API_EXTI_TRIGGER_RISING, IRQ_PRIO_MPU6050, IRQ_SUB_PRIO_MPU6050);
}

/****************************** NRF24L01 控制脚注册层 ************************/
/* NRF24L01 CE 控制脚配置表：把 HW_NRF24L01_CTRL_MAP 展开成 NRF24L01_CtrlConfig_t。 */
#define ENROLL_NRF24L01_CTRL_ITEM(cePort, cePin) \
	{ cePort, cePin },

static const NRF24L01_CtrlConfig_t s_nrfCtrlTable[] =
{
	HW_NRF24L01_CTRL_MAP(ENROLL_NRF24L01_CTRL_ITEM)
};
#undef ENROLL_NRF24L01_CTRL_ITEM

/* NRF24L01 CE 脚注册：登记板级 CE 控制引脚（PC5）。 */
void Enroll_NRF24L01_Register(void)
{
	NRF24L01_RegisterCtrl(s_nrfCtrlTable, HW_NRF24L01_CTRL_COUNT);
}
