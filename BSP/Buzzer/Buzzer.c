#include "Buzzer.h"

#include "pwm.h"
#include "LED.h"
#include "Delay.h"

/*
	无缘蜂鸣器-调用API层的PWM
*/
#define BUZZER_PWM_ARR ((1000000U / 2700U) - 1U)

void Buzzer_Init(void)
{
	/* 初始化时短鸣确认 PWM 输出正常，运行期由 Buzzer_On/Off 控制。 */
	API_PWM_Setcom(API_PWM_TIM3, API_PWM_CH4, BUZZER_PWM_ARR / 2U);
	Delay_ms(300U);
	Buzzer_Off();
	LED_Control(LED3, LED_LOW);
}

/* 固定约 2.7kHz、50% 占空比，适合无源蜂鸣器，调用不等待。 */
void Buzzer_On(void)
{
	API_PWM_Setcom(API_PWM_TIM3, API_PWM_CH4, BUZZER_PWM_ARR / 2U);
}

/* 比较值清零即静音，不停止 PWM 定时器。 */
void Buzzer_Off(void)
{
	API_PWM_Setcom(API_PWM_TIM3, API_PWM_CH4, 0U);
}
