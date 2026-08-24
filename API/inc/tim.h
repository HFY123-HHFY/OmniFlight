#ifndef __API_TIM_H
#define __API_TIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "f407_tim.h"

typedef enum
{
	API_TIM1 = 0U,
	API_TIM2 = 1U,
	API_TIM3 = 2U,
	API_TIM4 = 3U,
	API_TIM5 = 4U
} API_TIM_Id_t;

#define API_TIM_CORE_TIM2  (1U)
#define API_TIM_CORE_TIM3  (2U)
#define API_TIM_CORE_TIM4  (3U)
#define API_TIM_CORE_TIM5  (4U)

typedef struct
{
	API_TIM_Id_t id;
	uint8_t coreId;
} API_TIM_Config_t;

typedef void (*API_TIM_IrqHandler_t)(API_TIM_Id_t id);

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
} F407_TIM_View_t;

#define TIM2 ((F407_TIM_View_t *)0x40000000UL)
#define TIM3 ((F407_TIM_View_t *)0x40000400UL)
#define TIM4 ((F407_TIM_View_t *)0x40000800UL)
#define TIM5 ((F407_TIM_View_t *)0x40000C00UL)
#define TIM_SR_UIF (1UL << 0)

/*
 * 定时器初始化接口：
 * id 选择定时器实例，periodMs 指定中断周期（毫秒）。
 */
void API_TIM_Register(const API_TIM_Config_t *configTable, uint8_t count);
void API_TIM_RegisterIrqHandler(API_TIM_Id_t id, API_TIM_IrqHandler_t handler);
void API_TIM_Init(API_TIM_Id_t id, uint32_t periodMs);
void API_TIM_HandleIrqByCoreId(uint8_t coreId);

#ifdef __cplusplus
}
#endif

#endif /* __API_TIM_H */
