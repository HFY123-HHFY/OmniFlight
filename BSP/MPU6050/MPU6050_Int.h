#ifndef __MPU6050_INT_H
#define __MPU6050_INT_H

#include <stdint.h>

#include "sys.h"
#include "exti.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 供 MCU it 文件调用：处理某个 EXTI 线组（如 5~9 或 10~15）。 */
void MPU6050_EXTI_IRQHandlerGroup(uint8_t startLine, uint8_t endLine);
void MPU6050_EXTI_Callback(API_EXTI_Id_t id, void *userData);

extern float Pitch, Roll, Yaw;	        /* Pitch：俯仰角，Roll：横滚角，Yaw：偏航角 */
extern short gyrox, gyroy, gyroz;       /*         角速度,x轴、y轴、z轴            */
extern short aacx, aacy, aacz;          /*        加速度 ,x轴、y轴、z轴           */

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_INT_H */
