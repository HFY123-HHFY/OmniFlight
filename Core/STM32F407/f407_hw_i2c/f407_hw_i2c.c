#include "f407_hw_i2c.h"

#include "f407_gpio.h"
#include "Delay.h"

/*
 * f407_hw_i2c.c — STM32F407 硬件 I2C1 驱动
 *
 * 用 I2C1 外设 (PB8=SCL, PB9=SDA, AF4) 实现 I2C_HAL_Ops。
 * APB1=42MHz, 支持 100/400 kHz。
 */

/* ===================================================================
 * I2C 寄存器
 * =================================================================== */
typedef struct
{
	volatile uint32_t CR1;   volatile uint32_t CR2;
	volatile uint32_t OAR1;  volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;   volatile uint32_t SR2;
	volatile uint32_t CCR;   volatile uint32_t TRISE;
	volatile uint32_t FLTR;
} I2C_Regs_t;

#define I2C1         ((I2C_Regs_t *)0x40005400UL)

#define CR1_PE       (1UL << 0)
#define CR1_START    (1UL << 8)
#define CR1_STOP     (1UL << 9)
#define CR1_ACK      (1UL << 10)
#define CR1_SWRST    (1UL << 15)

#define SR1_SB       (1UL << 0)
#define SR1_ADDR     (1UL << 1)
#define SR1_BTF      (1UL << 2)
#define SR1_RXNE     (1UL << 6)
#define SR1_TXE      (1UL << 7)
#define SR1_AF       (1UL << 10)
#define SR1_BERR     (1UL << 8)
#define SR1_ARLO     (1UL << 9)
#define SR1_OVR      (1UL << 11)

#define SR2_BUSY     (1UL << 1)
#define SR2_TRA      (1UL << 2)

#define CCR_FS       (1UL << 15)

/* ===================================================================
 * 状态
 * =================================================================== */
static I2C_Regs_t     *I2C;
static F407_GPIO_Regs_t *sclGpio, *sdaGpio;
static uint32_t         sclPin, sdaPin;
static uint32_t         speedKhz = 100U;

/* 上一个发送字节的 ACK 结果（0=ACK, 1=NACK），由 SendByte 写入，WaitAck 返回 */
static uint8_t          last_ack = 0U;

/* ===================================================================
 * GPIO 配置
 * =================================================================== */

static uint32_t pin_idx(uint32_t pin)
{
	uint32_t i = 0U;
	while (!(pin & 1U)) { pin >>= 1U; i++; }
	return i;
}

static void config_af(F407_GPIO_Regs_t *reg, uint32_t pin)
{
	uint32_t idx = pin_idx(pin);
	uint32_t sh  = idx * 2U;
	F407_GPIO_EnablePortClock(reg);
	reg->MODER   = (reg->MODER   & ~(0x3UL << sh))  | (0x2UL << sh);
	reg->OTYPER  |= (1UL << idx);
	reg->OSPEEDR = (reg->OSPEEDR & ~(0x3UL << sh))  | (0x2UL << sh);
	reg->PUPDR   = (reg->PUPDR   & ~(0x3UL << sh))  | (0x1UL << sh);
	if (idx < 8U) reg->AFRL = (reg->AFRL & ~(0xFUL << (idx * 4U))) | (4UL << (idx * 4U));
	else          reg->AFRH = (reg->AFRH & ~(0xFUL << ((idx-8U) * 4U))) | (4UL << ((idx-8U) * 4U));
}

static void clock_enable(void)
{
	*(volatile uint32_t *)0x40023840UL |= (1UL << 21U);
	__asm volatile("dsb");
}

static void soft_reset(void)
{
	I2C->CR1 |= CR1_SWRST;
	Delay_us(10U);
	I2C->CR1 &= ~CR1_SWRST;
}

static void config_timing(uint32_t khz)
{
	uint32_t pclk1 = 42000000U, ccr, trise;
	if (khz <= 100U) {
		ccr = pclk1 / (khz * 2000U); if (ccr < 4U) ccr = 4U;
		trise = (pclk1 / 1000000U) + 1U;
		I2C->CCR = ccr;
	} else {
		ccr = pclk1 / (khz * 3000U); if (ccr < 1U) ccr = 1U;
		trise = (pclk1 / 1000000U) * 300U / 1000U + 1U;
		I2C->CCR = CCR_FS | ccr;
	}
	I2C->TRISE = trise;
}

/* 清除所有错误/状态——每次 Start 前调用 */
static void clear_flags(void)
{
	uint32_t sr1 = I2C->SR1;
	(void)sr1;
	if (sr1 & SR1_AF)   I2C->SR1 &= ~SR1_AF;
	if (sr1 & SR1_ARLO) I2C->SR1 &= ~SR1_ARLO;
	if (sr1 & SR1_BERR) I2C->SR1 &= ~SR1_BERR;
	if (sr1 & SR1_OVR)  I2C->SR1 &= ~SR1_OVR;
}

/* ===================================================================
 * Ops 实现
 * =================================================================== */

static void hw_init(void *sclPort, uint32_t sclPin_, uint32_t sclIomux,
                    void *sdaPort, uint32_t sdaPin_, uint32_t sdaIomux)
{
	(void)sclIomux; (void)sdaIomux;
	I2C     = I2C1;
	sclGpio = (F407_GPIO_Regs_t *)sclPort;
	sdaGpio = (F407_GPIO_Regs_t *)sdaPort;
	sclPin  = sclPin_;
	sdaPin  = sdaPin_;

	clock_enable();
	config_af(sclGpio, sclPin);
	config_af(sdaGpio, sdaPin);

	soft_reset();
	config_timing(speedKhz);
	I2C->CR2 = 42U;
	I2C->CR1 = CR1_PE;
}

static void hw_select(void *sp, uint32_t sP, uint32_t sI, void *dp, uint32_t dP, uint32_t dI)
{
	(void)sI; (void)dI;
	/* 只更新引脚引用，不重初始化 — hw_init 已在系统启动时完成 */
	I2C     = I2C1;
	sclGpio = (F407_GPIO_Regs_t *)sp;
	sdaGpio = (F407_GPIO_Regs_t *)dp;
	sclPin  = sP;
	sdaPin  = dP;
}

/* ── Start ── */
static void hw_start(void)
{
	uint32_t to;

	/* 确保总线空闲 */
	to = 5000U;
	while ((I2C->SR2 & SR2_BUSY) && --to) {}

	/* 清除残留错误标志 */
	clear_flags();

	/* 产生 START */
	I2C->CR1 |= CR1_START;

	to = 5000U;
	while (!(I2C->SR1 & SR1_SB) && --to) {}

	last_ack = 0U;
}

/* ── Stop ── */
static void hw_stop(void)
{
	I2C->CR1 |= CR1_STOP;
	uint32_t to = 5000U;
	while ((I2C->SR2 & SR2_BUSY) && --to) {}
}

/* ── SendByte ──
 *
 * 地址阶段 (SB=1): 写 DR → 等 ADDR → 硬件自动检测 AF → 读 SR1,SR2 清 ADDR
 *                 同时捕获 AF 状态写入 last_ack
 * 数据阶段 (SB=0): 等 TXE → 写 DR → 等 BTF → 捕获 AF 状态
 */
static void hw_send_byte(uint8_t data)
{
	uint32_t to;

	if (I2C->SR1 & SR1_SB)
	{
		/* ── 地址阶段 ── */
		I2C->DR = data;

		to = 5000U;
		while (!(I2C->SR1 & SR1_ADDR) && --to) {}

		/* 捕获 AF：ADDR 和 AF 在同一时刻被硬件置位 */
		last_ack = (I2C->SR1 & SR1_AF) ? 1U : 0U;

		/* 读 SR1 再 SR2 清 ADDR */
		(void)I2C->SR1;
		(void)I2C->SR2;
	}
	else
	{
		/* ── 数据阶段 ── */
		to = 5000U;
		while (!(I2C->SR1 & SR1_TXE) && --to) {}

		I2C->DR = data;

		to = 5000U;
		while (!(I2C->SR1 & SR1_BTF) && --to) {}

		/* BTF 和 AF 同时置位 */
		last_ack = (I2C->SR1 & SR1_AF) ? 1U : 0U;
	}
}

/* ── WaitAck: 返回 SendByte 捕获的 ACK 结果 ── */
static uint8_t hw_wait_ack(void)
{
	if (last_ack)
	{
		I2C->SR1 &= ~SR1_AF;
		return 1U;
	}
	return 0U;
}

/* ── ReceiveByte ── */
static uint8_t hw_recv_byte(uint8_t ack)
{
	uint32_t to;

	if (ack) I2C->CR1 |=  CR1_ACK;
	else     I2C->CR1 &= ~CR1_ACK;

	to = 5000U;
	while (!(I2C->SR1 & SR1_RXNE) && --to) {}

	return (uint8_t)I2C->DR;
}

static void hw_ack(void)  {}
static void hw_nack(void) {}

static void hw_set_speed(uint32_t khz)
{
	if (khz == speedKhz) return;  /* 速率未变，跳过 PE 开关 */
	speedKhz = khz;
	I2C->CR1 &= ~CR1_PE;
	config_timing(khz);
	I2C->CR1 |= CR1_PE;
}

static void hw_delay_off(void) {}
static void hw_delay_on(void)  {}

/* ===================================================================
 * 操作表
 * =================================================================== */
I2C_HAL_Ops g_i2c_hal_ops_hw = {
	hw_init,   hw_select,
	hw_start,  hw_stop,
	hw_send_byte, hw_recv_byte,
	hw_ack,    hw_nack,    hw_wait_ack,
	hw_set_speed,
	hw_delay_off, hw_delay_on,
};
