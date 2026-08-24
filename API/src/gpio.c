#include "gpio.h"
#include "f407_gpio.h"

void API_GPIO_InitOutput(void *port, uint32_t pin)
{
	F407_GPIO_InitOutput(port, pin);
}

void API_GPIO_InitInput(void *port, uint32_t pin)
{
	F407_GPIO_InitInput(port, pin);
}

void API_GPIO_InitInputPullUp(void *port, uint32_t pin)
{
	F407_GPIO_InitInputPullUp(port, pin);
}

void API_GPIO_Write(void *port, uint32_t pin, uint8_t level)
{
	F407_GPIO_Write(port, pin, level);
}

uint8_t API_GPIO_Read(void *port, uint32_t pin)
{
	return F407_GPIO_Read(port, pin);
}
