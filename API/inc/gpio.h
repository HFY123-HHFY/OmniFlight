#ifndef __API_GPIO_H
#define __API_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "f407_gpio.h"

/* 配置 GPIO 为推挽输出。 */
void API_GPIO_InitOutput(void *port, uint32_t pin);
/* 配置 GPIO 为输入模式。 */
void API_GPIO_InitInput(void *port, uint32_t pin);
/* 配置 GPIO 为上拉输入模式。 */
void API_GPIO_InitInputPullUp(void *port, uint32_t pin);
/* GPIO 输出电平控制。 */
void API_GPIO_Write(void *port, uint32_t pin, uint8_t level);
/* GPIO 输入电平读取。 */
uint8_t API_GPIO_Read(void *port, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __API_GPIO_H */
