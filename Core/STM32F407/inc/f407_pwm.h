#ifndef __F407_PWM_H
#define __F407_PWM_H

#include <stdint.h>

#include "f407_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 配置 PWM 输出引脚为复用功能。 */
void F407_PWM_ConfigPin(void *port, uint16_t pin, uint8_t timId);
/* 初始化指定定时器的 ARR/PSC 并启动。 */
void F407_PWM_InitTimer(uint8_t timId, uint16_t arr, uint16_t psc);
/* 设置指定通道的 CCR 值（首次调用自动配置 PWM 模式 1 + 预装载）。 */
void F407_PWM_SetCCR(uint8_t timId, uint8_t channel, uint16_t ccr);

/*
 * 获取定时器外设基地址。
 * 用于 DMA 外设地址配置或高级协议层直接寄存器访问。
 */
uint32_t F407_PWM_GetTimerBase(uint8_t timId);

/*
 * 配置高级定时器（TIM1）的 DMA burst。
 * dba: DMA 基地址偏移（目标寄存器偏移 / 4，如 CCR1 = 0x0D）
 * burstLen: burst 传输次数编码（0=1次, 1=2次, ..., 3=4次）
 * 仅对 TIM1 有效，其他定时器调用无操作。
 */
void F407_PWM_ConfigDMABurst(uint8_t timId, uint8_t dba, uint8_t burstLen);

/* 使能定时器 Update 事件的 DMA 请求（DIER.UDE）。 */
void F407_PWM_EnableUpdateDMA(uint8_t timId);

/* 直接设置定时器 ARR 值（不改其他寄存器，用于运行时调频）。 */
void F407_PWM_SetARR(uint8_t timId, uint16_t arr);

/* 触发定时器更新事件（EGR.UG），将预装载值同步到影子寄存器。 */
void F407_PWM_ForceUpdate(uint8_t timId);

#ifdef __cplusplus
}
#endif

#endif /* __F407_PWM_H */
