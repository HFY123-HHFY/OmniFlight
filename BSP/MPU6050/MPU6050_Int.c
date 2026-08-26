#include "MPU6050_Int.h"
#include "MPU6050.h"
#include "Control_Task/Control_Task.h"

/*
    * MPU6050 外部中断处理代码
     * 1) 通过 Enroll 层注册 EXTI 线资源，优先级由 IrqPriority.h 统一管理；
     * 2) 中断服务函数调用 MPU6050_EXTI_IRQHandlerGroup，根据线号分组处理；
     * 3) 中断回调 MPU6050_EXTI_Callback 通过 ControlTask_NotifyMpuIsr()
     *    给 RTOS 信号量，唤醒 SensorTask（200Hz）读取传感器。
     * 注意：MPU6050 的 INT 引脚默认是低电平有效的，因此建议配置为上升沿触发。

 */

float Pitch = 0.0f, Roll = 0.0f, Yaw = 0.0f;	        /* Pitch：俯仰角，Roll：横滚角，Yaw：偏航角 */
short gyrox = 0, gyroy = 0, gyroz = 0;      /*         角速度,x轴、y轴、z轴            */
short aacx = 0, aacy = 0, aacz = 0;          /*        加速度 ,x轴、y轴、z轴           */
/*short短整型，16位有符号整数，范围-32768~32767，单位：m/s^2, %hd*/

void MPU6050_EXTI_IRQHandlerGroup(uint8_t startLine, uint8_t endLine)
{
	API_EXTI_HandleIrqByLineGroup(startLine, endLine);
}

/* MPU6050 外部中断回调函数 */
void MPU6050_EXTI_Callback(API_EXTI_Id_t id, void *userData)
{
	(void)id;
	(void)userData;
	ControlTask_NotifyMpuIsr();  /* 通知 SensorTask 读传感器 */
}
