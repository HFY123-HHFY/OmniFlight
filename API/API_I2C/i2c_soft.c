#include "i2c_hal.h"
#include "soft_i2c_hal.h"

/*
 * i2c_soft.c — 软件 I2C 实现
 *
 * 用 GPIO 翻转模拟 I2C 时序，填一份 I2C_HAL_Ops 合同。
 * GPIO 操作委托给 soft_i2c_hal.h 原语（各 MCU 提供实现）。
 * 本文件本身跨平台，不依赖任何特定 MCU。
 */

/* ACK 等待超时轮次 */
#define SOFT_I2C_ACK_TIMEOUT (5000U)

/* ── 内部封装：soft_i2c_hal 原语 ─────────────────────────── */

static void s_w_scl(uint8_t bit) { soft_i2c_hal_w_scl(bit); }
static void s_w_sda(uint8_t bit) { soft_i2c_hal_w_sda(bit); }
static uint8_t s_r_sda(void)      { return soft_i2c_hal_r_sda(); }
static void s_sda_in(void)        { soft_i2c_hal_set_sda_input(); }
static void s_sda_out(void)       { soft_i2c_hal_set_sda_output(); }
static void s_delay(uint32_t us)  { soft_i2c_hal_delay_us(us); }

/* ── 协议函数 ─────────────────────────────────────────── */

static void soft_i2c_start(void)
{
	s_sda_out();
	s_w_sda(1U);
	s_w_scl(1U);
	s_delay(4U);
	s_w_sda(0U);
	s_delay(4U);
	s_w_scl(0U);
}

static void soft_i2c_stop(void)
{
	s_sda_out();
	s_w_scl(0U);
	s_w_sda(0U);
	s_delay(4U);
	s_w_scl(1U);
	s_w_sda(1U);
	s_delay(4U);
}

static uint8_t soft_i2c_wait_ack(void)
{
	uint16_t ErrTime = 0U;

	s_sda_in();
	s_delay(1U);
	s_w_scl(1U);
	s_delay(1U);

	while (s_r_sda())
	{
		ErrTime++;
		if (ErrTime > SOFT_I2C_ACK_TIMEOUT)
		{
			soft_i2c_stop();
			return 1U;
		}
	}

	s_w_scl(0U);
	return 0U;
}

static void soft_i2c_ack(void)
{
	s_w_scl(0U);
	s_sda_out();
	s_w_sda(0U);
	s_delay(2U);
	s_w_scl(1U);
	s_delay(2U);
	s_w_scl(0U);
}

static void soft_i2c_nack(void)
{
	s_w_scl(0U);
	s_sda_out();
	s_w_sda(1U);
	s_delay(2U);
	s_w_scl(1U);
	s_delay(2U);
	s_w_scl(0U);
}

static void soft_i2c_send_byte(uint8_t data)
{
	uint8_t i;

	s_sda_out();
	for (i = 0U; i < 8U; i++)
	{
		s_w_sda((data & 0x80U) >> 7U);
		data <<= 1U;
		s_delay(2U);
		s_w_scl(1U);
		s_delay(2U);
		s_w_scl(0U);
		s_delay(2U);
	}
}

static uint8_t soft_i2c_receive_byte(uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0U;

	s_sda_in();
	for (i = 0U; i < 8U; i++)
	{
		s_w_scl(0U);
		s_delay(2U);
		s_w_scl(1U);
		data <<= 1U;
		if (s_r_sda() != 0U)
		{
			data++;
		}
		s_delay(1U);
	}

	if (ack != 0U)
	{
		soft_i2c_ack();
	}
	else
	{
		soft_i2c_nack();
	}

	return data;
}

/* ── Ops 表实例 ──────────────────────────────────────── */

I2C_HAL_Ops g_i2c_hal_ops_soft = {
	soft_i2c_hal_init,         /* Init      — 委托平台 GPIO 初始化 */
	soft_i2c_hal_init,         /* SelectBus — 复用 Init 重新缓存寄存器 */
	soft_i2c_start,
	soft_i2c_stop,
	soft_i2c_send_byte,
	soft_i2c_receive_byte,
	soft_i2c_ack,
	soft_i2c_nack,
	soft_i2c_wait_ack,
	soft_i2c_hal_set_speed,
	soft_i2c_hal_delay_off,
	soft_i2c_hal_delay_on,
};
