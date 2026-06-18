#ifndef __SOFT_I2C_HAL_H
#define __SOFT_I2C_HAL_H

#include <stdint.h>

/*
 * soft_i2c_hal.h — GPIO 原语
 *
 * 只做一件事：操作 SCL/SDA 两个引脚的高低电平 + 微秒延时。
 * 不关心 I2C 协议（Start/Stop/ACK 等），那是 i2c_soft.c 的事。
 *
 * 各 MCU 的 Core 层各提供一份实现：
 *   f407_soft_i2c.c  /  f103_soft_i2c.c  /  G3507_soft_i2c.c
 */

void     soft_i2c_hal_init(void *sclPort, uint32_t sclPin, uint32_t sclIomux,
                           void *sdaPort, uint32_t sdaPin, uint32_t sdaIomux);
void     soft_i2c_hal_w_scl(uint8_t bit);
void     soft_i2c_hal_w_sda(uint8_t bit);
uint8_t  soft_i2c_hal_r_sda(void);
void     soft_i2c_hal_set_sda_input(void);
void     soft_i2c_hal_set_sda_output(void);
void     soft_i2c_hal_delay_us(uint32_t us);
void     soft_i2c_hal_set_speed(uint32_t speedKhz);
void     soft_i2c_hal_delay_off(void);
void     soft_i2c_hal_delay_on(void);

#endif
