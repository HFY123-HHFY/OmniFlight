#ifndef __I2C_HAL_H
#define __I2C_HAL_H

#include <stdint.h>

/*
 * i2c_hal.h — 合同
 *
 * 定义一组函数指针（Start/Stop/SendByte/...），
 * 软硬件各提供一份实现，API_I2C.c 只认这份合同。
 *
 * 类比：这是一个 C 语言版的 "interface"。
 */

typedef const struct I2C_HAL_Ops
{
	void (*Init)(void *sclPort, uint32_t sclPin, uint32_t sclIomux,
	             void *sdaPort, uint32_t sdaPin, uint32_t sdaIomux);
	void (*SelectBus)(void *sclPort, uint32_t sclPin, uint32_t sclIomux,
	                  void *sdaPort, uint32_t sdaPin, uint32_t sdaIomux);

	void     (*Start)(void);
	void     (*Stop)(void);
	void     (*SendByte)(uint8_t data);
	uint8_t  (*ReceiveByte)(uint8_t ack);
	void     (*Ack)(void);
	void     (*NAck)(void);
	uint8_t  (*WaitAck)(void);

	void     (*SetSpeed)(uint32_t speedKhz);
	void     (*DelayOff)(void);
	void     (*DelayOn)(void);
} I2C_HAL_Ops;

/* 当前挂载的实现 — 由 Enroll_I2C_Register() 设置 */
extern I2C_HAL_Ops *g_i2c_hal_ops;

#endif
