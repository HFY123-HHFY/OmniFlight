#include "Buzzer.h"

#include "pwm.h"
#include "LED.h"
#include "Delay.h"

/*
	无缘蜂鸣器-调用API层的PWM
*/

void Buzzer_Init(void)
{
	/* ARR = (1MHz / 2700Hz) - 1 ≈ 369, 取 50% 占空比产生最大方波振幅 */
	uint16_t arr = (1000000U / 2700U) - 1U;

	API_PWM_Setcom(API_PWM_TIM3, API_PWM_CH4, arr / 2U);  /* 50% duty */
	Delay_ms(300U);
	API_PWM_Setcom(API_PWM_TIM3, API_PWM_CH4, 0U);        /* 静音 */
	LED_Control(LED3, LED_LOW);
}
