#ifndef __407_HW_CONFIG_H
#define __407_HW_CONFIG_H

#include "f407_gpio.h" /* GPIO_Pin_x + GPIOA/B 原语，其余枚举值在 Enroll.c 展开时由 Enroll_Internal.h 提供 */

/*
 * 407_hw_config.h 板级硬件映射宏
 */

/*
LED1 绿 LED2 红 LED3 蓝
*/
#define HW_LED_MAP(X) \
	X(LED1, GPIOE, GPIO_Pin_2) \
	X(LED2, GPIOE, GPIO_Pin_3) \
	X(LED3, GPIOE, GPIO_Pin_4)

/* 板载调试 USART1 引脚定义：TX=PA9，RX=PA10 */
#define HW_USART1_TX_PORT GPIOA
#define HW_USART1_TX_PIN  GPIO_Pin_9
#define HW_USART1_RX_PORT GPIOA
#define HW_USART1_RX_PIN  GPIO_Pin_10

/* 无线串口 USART3 引脚定义：TX=PD8，RX=PD9 */
#define HW_USART3_TX_PORT GPIOD
#define HW_USART3_TX_PIN  GPIO_Pin_8
#define HW_USART3_RX_PORT GPIOD
#define HW_USART3_RX_PIN  GPIO_Pin_9

/* USART 板级映射：当前板子注册 2 路串口 */
#define HW_USART_MAP(X) \
	X(API_USART1, API_USART_CORE_USART1, HW_USART1_TX_PORT, HW_USART1_TX_PIN, HW_USART1_RX_PORT, HW_USART1_RX_PIN) \
	X(API_USART3, API_USART_CORE_USART3, HW_USART3_TX_PORT, HW_USART3_TX_PIN, HW_USART3_RX_PORT, HW_USART3_RX_PIN)

/* TIM中断服务函数  注册了2路 */
#define HW_TIM_MAP(X) \
	X(API_TIM1, API_TIM_CORE_TIM3) \
	X(API_TIM2, API_TIM_CORE_TIM2)
	
/* PWM 板级映射：DShot 已接管 TIM1 四路电机输出，API PWM 层不再管理 TIM1。
   保留空映射以维持 Enroll.c / API pwm.c 编译兼容。 */
#define HW_PWM_MAP(X)

/* 软件 I2C1 引脚定义：SCL=PB8，SDA=PB9（MPU/QMC/BMP）*/
#define HW_I2C1_SCL_PORT GPIOB
#define HW_I2C1_SCL_PIN  GPIO_Pin_8
#define HW_I2C1_SDA_PORT GPIOB
#define HW_I2C1_SDA_PIN  GPIO_Pin_9

/* I2C 板级映射 */
#define HW_I2C_MAP(X) \
	X(API_I2C1, HW_I2C1_SCL_PORT, HW_I2C1_SCL_PIN, HW_I2C1_SDA_PORT, HW_I2C1_SDA_PIN, 0, 0)

/* 软件 SPI1 引脚定义：给 NRF24L01 */
#define HW_SPI1_SCK_PORT  GPIOA
#define HW_SPI1_SCK_PIN   GPIO_Pin_5

#define HW_SPI1_MOSI_PORT GPIOA
#define HW_SPI1_MOSI_PIN  GPIO_Pin_7

#define HW_SPI1_MISO_PORT GPIOA
#define HW_SPI1_MISO_PIN  GPIO_Pin_6

#define HW_SPI1_CS_PORT   GPIOC
#define HW_SPI1_CS_PIN    GPIO_Pin_4

/* SPI 板级映射：注册 1 路软件 SPI */
#define HW_SPI_MAP(X) \
	X(API_SPI1, HW_SPI1_CS_PORT, HW_SPI1_CS_PIN, \
	  HW_SPI1_SCK_PORT, HW_SPI1_SCK_PIN, \
	  HW_SPI1_MOSI_PORT, HW_SPI1_MOSI_PIN, \
	  HW_SPI1_MISO_PORT, HW_SPI1_MISO_PIN, 0, 0, 0, 0)

/* NRF24L01 控制引脚定义：CE=PC5 */
#define HW_NRF24L01_CE_PORT GPIOC
#define HW_NRF24L01_CE_PIN  GPIO_Pin_5

/* NRF24L01 控制引脚映射：注册 1 组 CE */
#define HW_NRF24L01_CTRL_MAP(X) \
	X(HW_NRF24L01_CE_PORT, HW_NRF24L01_CE_PIN)

/* DShot 电机映射：逻辑电机通道 → 引脚。
   用于 BSP/Dshot 模块配置 4 路 TIM1 PWM 输出引脚。
   修改引脚只需改此宏，DShot 代码无需改动。 */
#define HW_DSHOT_MOTOR_MAP(X) \
    X(1U, GPIOE, GPIO_Pin_9) \
    X(2U, GPIOE, GPIO_Pin_11) \
    X(3U, GPIOE, GPIO_Pin_13) \
    X(4U, GPIOE, GPIO_Pin_14)

/* 当前板子上注册了 3 个 LED */
#define HW_LED_COUNT  3U
/* 当前板子上注册了 1 个按键 */
#define HW_KEY_COUNT  1U
/* 当前板子上注册了 2 路 USART */
#define HW_USART_COUNT  2U
/* 当前板子上注册了 2 路 TIM中断服务函数 */
#define HW_TIM_COUNT  2U
/* 当前板子上注册了 0 路 PWM 通道（改由 BSP/Dshot 直接管理 TIM1） */
#define HW_PWM_COUNT  0U
/* 当前板子上注册了 4 路 ADC 通道 */
#define HW_ADC_COUNT  4U
/* 当前板子上注册了 1 路软件 I2C */
#define HW_I2C_COUNT  1U
/* 当前板子上注册了 1 路软件 SPI */
#define HW_SPI_COUNT  1U
/* 当前板子上注册了 1 组 NRF24L01 控制引脚 */
#define HW_NRF24L01_CTRL_COUNT  1U
/* 当前板子上注册了 4 路 DShot 电机通道 */
#define HW_DSHOT_MOTOR_COUNT  4U
/* DShot 使用的硬件定时器编号（TIM1 = 高级定时器，支持 DMA burst 和 MOE） */
#define HW_DSHOT_TIM_ID       1U

/* MPU6050 INT 板级映射：仅维护引脚资源，优先级策略由 sys.c 统一管理 */
#define HW_MPU6050_INT_PORT             GPIOE
#define HW_MPU6050_INT_PIN              GPIO_Pin_7

#endif /* __407_HW_CONFIG_H */
