#include "API_I2C.h"
#include "i2c_hal.h"

#include "Delay.h"
#include "My_Usart/My_Usart.h"

/* ===================== 模块状态变量 ===================== */

/*
 * 注册表: 保存 Enroll 层下发的所有总线映射。
 */
static const API_I2C_Config_t *s_i2cTable;
static uint8_t s_i2cCount;
static volatile API_I2C_BusId_t s_activeBusId = API_I2C1;
static volatile API_I2C_SpeedTypeDef s_i2cSpeed = API_I2C_SPEED_100K;

/* 全局操作表指针 — 由 Enroll 层设置。默认指向软件 I2C。 */
I2C_HAL_Ops *g_i2c_hal_ops = 0;

/* ===================== 内部辅助 ===================== */

/*
 * 按 busId 查找已注册的总线配置表。
 */
static const API_I2C_Config_t *API_I2C_GetConfigById(API_I2C_BusId_t busId)
{
	uint8_t i;

	if ((s_i2cTable == 0) || (s_i2cCount == 0U))
	{
		return 0;
	}

	for (i = 0U; i < s_i2cCount; i++)
	{
		if (s_i2cTable[i].id == (uint8_t)busId)
		{
			return &s_i2cTable[i];
		}
	}

	return 0;
}

/*
 * 安全调用 ops 宏：ops 未初始化时直接返回避免崩溃。
 */
#define I2C_OPS_CALL_VOID(fn, ...)          do { if (g_i2c_hal_ops && g_i2c_hal_ops->fn) g_i2c_hal_ops->fn(__VA_ARGS__); } while(0)
#define I2C_OPS_CALL_RET(fn, def, ...)      ((g_i2c_hal_ops && g_i2c_hal_ops->fn) ? g_i2c_hal_ops->fn(__VA_ARGS__) : (def))

/* ===================== 公共 API ===================== */

/*
 * 注册板级 I2C 资源表。
 */
void API_I2C_Register(const API_I2C_Config_t *configTable, uint8_t count)
{
	s_i2cTable = configTable;
	s_i2cCount = count;

	if ((configTable != 0) && (count > 0U))
	{
		s_activeBusId = (API_I2C_BusId_t)configTable[0].id;
		I2C_OPS_CALL_VOID(Init,
		                  configTable[0].sclPort, configTable[0].sclPin, configTable[0].sclIomux,
		                  configTable[0].sdaPort, configTable[0].sdaPin, configTable[0].sdaIomux);
	}
	else
	{
		s_activeBusId = API_I2C1;
	}
}

/*
 * 选择当前操作的 I2C 总线。
 */
void API_I2C_SelectBus(API_I2C_BusId_t busId)
{
	const API_I2C_Config_t *cfg;

	cfg = API_I2C_GetConfigById(busId);
	if (cfg != 0)
	{
		s_activeBusId = busId;
		I2C_OPS_CALL_VOID(SelectBus,
		                  cfg->sclPort, cfg->sclPin, cfg->sclIomux,
		                  cfg->sdaPort, cfg->sdaPin, cfg->sdaIomux);
	}

	/* 总线切换后恢复默认延时 */
	I2C_OPS_CALL_VOID(DelayOn);
}

void API_I2C_DelayOff(void)
{
	I2C_OPS_CALL_VOID(DelayOff);
}

void API_I2C_DelayOn(void)
{
	I2C_OPS_CALL_VOID(DelayOn);
}

/*
 * 设置 I2C 速率档位。
 */
void API_I2C_SetSpeed(API_I2C_SpeedTypeDef speed)
{
	uint32_t speedKhz;

	switch (speed)
	{
	case API_I2C_SPEED_400K: s_i2cSpeed = API_I2C_SPEED_400K; speedKhz = 400U; break;
	case API_I2C_SPEED_200K: s_i2cSpeed = API_I2C_SPEED_200K; speedKhz = 200U; break;
	case API_I2C_SPEED_50K:  s_i2cSpeed = API_I2C_SPEED_50K;  speedKhz = 50U;  break;
	default:                 s_i2cSpeed = API_I2C_SPEED_100K; speedKhz = 100U; break;
	}
	I2C_OPS_CALL_VOID(SetSpeed, speedKhz);
}

API_I2C_SpeedTypeDef API_I2C_GetSpeed(void)
{
	return s_i2cSpeed;
}

/* ===================== 初始化 ===================== */

void API_I2C_Init(void)
{
	uint8_t i;
	API_I2C_BusId_t prevBus;

	if ((s_i2cTable == 0) || (s_i2cCount == 0U))
	{
		return;
	}

	prevBus = s_activeBusId;

	for (i = 0U; i < s_i2cCount; i++)
	{
		const API_I2C_Config_t *cfg = &s_i2cTable[i];
		I2C_OPS_CALL_VOID(Init,
		                  cfg->sclPort, cfg->sclPin, cfg->sclIomux,
		                  cfg->sdaPort, cfg->sdaPin, cfg->sdaIomux);
	}

	/* 恢复活跃总线 */
	{
		const API_I2C_Config_t *cfg = API_I2C_GetConfigById(prevBus);
		if (cfg != 0)
		{
			I2C_OPS_CALL_VOID(Init,
			                  cfg->sclPort, cfg->sclPin, cfg->sclIomux,
			                  cfg->sdaPort, cfg->sdaPin, cfg->sdaIomux);
		}
	}

	I2C_OPS_CALL_VOID(SetSpeed, 100U);
	I2C_OPS_CALL_VOID(DelayOn);
}

/* ===================== 协议层 ===================== */

void API_I2C_Start(void)
{
	I2C_OPS_CALL_VOID(Start);
}

void API_I2C_Stop(void)
{
	I2C_OPS_CALL_VOID(Stop);
}

uint8_t API_I2C_Wait_Ack(void)
{
	return I2C_OPS_CALL_RET(WaitAck, 1U);
}

void API_I2C_Ack(void)
{
	I2C_OPS_CALL_VOID(Ack);
}

void API_I2C_NAck(void)
{
	I2C_OPS_CALL_VOID(NAck);
}

void API_I2C_SendByte(uint8_t Byte)
{
	I2C_OPS_CALL_VOID(SendByte, Byte);
}

uint8_t API_I2C_ReceiveByte(unsigned char Ack)
{
	return I2C_OPS_CALL_RET(ReceiveByte, 0U, Ack);
}

/* ===================== I2C 扫描 (调试用) ===================== */

static void App_I2C_ScanBus(API_I2C_BusId_t busId)
{
	uint8_t addr;
	uint8_t foundCount;
	const API_I2C_Config_t *config;
	API_I2C_BusId_t prevBusId;
	API_I2C_SpeedTypeDef prevSpeed;

	config = API_I2C_GetConfigById(busId);
	if (config == 0)
	{
		return;
	}

	prevBusId = s_activeBusId;
	prevSpeed = s_i2cSpeed;

	API_I2C_SelectBus(busId);
	API_I2C_SetSpeed(API_I2C_SPEED_100K);

	foundCount = 0U;
	usart_printf(USART1, "\r\n[I2C][API_I2C%u] scan start\r\n", (unsigned int)((uint8_t)busId + 1U));

	for (addr = 1U; addr < 0x7FU; addr++)
	{
		API_I2C_Start();
		API_I2C_SendByte((uint8_t)(addr << 1));
		if (API_I2C_Wait_Ack() == 0U)
		{
			foundCount++;
			usart_printf(USART1, "[I2C][API_I2C%u] found: 0x%02X\r\n", (unsigned int)((uint8_t)busId + 1U), addr);
		}
		API_I2C_Stop();
		Delay_ms(1U);
	}

	usart_printf(USART1, "[I2C][API_I2C%u] scan done, count=%u\r\n", (unsigned int)((uint8_t)busId + 1U), foundCount);

	API_I2C_SelectBus(prevBusId);
	API_I2C_SetSpeed(prevSpeed);
}

void App_I2C_ScanOnce(void)
{
	uint8_t i;

	if ((s_i2cTable == 0) || (s_i2cCount == 0U))
	{
		usart_printf(USART1, "\r\n[I2C] scan skipped: no bus registered\r\n");
		return;
	}

	for (i = 0U; i < s_i2cCount; i++)
	{
		App_I2C_ScanBus((API_I2C_BusId_t)s_i2cTable[i].id);
	}
}
