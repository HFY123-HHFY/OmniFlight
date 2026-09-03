#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "MPU6050_Int.h"
#include "pwm.h"
#include "TB6612.h"
#include "Motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ±2000dps 量程下 MPU6050 陀螺灵敏度：16.4 LSB/(deg/s) */
#define GYRO_SENS_2000DPS (16.4f)

/* PID 输出加载到电机前的总限幅。 */
#define MOTOR_MIX_LIMIT (2047.0f)

/* 目标姿态/高度。 */
extern float Target_Pitch;
extern float Target_Roll;
extern float Target_Yaw;

/* 目标偏航角速度 (deg/s)。当前恒 0 用于消除自旋，后续由遥控器写入。 */
extern float Target_Yaw_Rate;

/* 外环 PID：角度。 */
extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;

/* 内环 PID：角速度。 */
extern PID_TypeDef pid_rate_pitch;
extern PID_TypeDef pid_rate_roll;
extern PID_TypeDef pid_rate_yaw;

/* 高度环 */
extern PID_TypeDef pid_alt;

/*
 * 控制初始化：
 * 1) 初始化外环/内环 PID（含限幅）
 * 2) 初始化串级控制对象
 * 3) 初始化陀螺低通滤波器
 */
void PID_Contorl_Init(void);

/*
 * 陀螺零偏校准（上电后调用一次，飞行器必须静止）。
 * samples: 采样点数（建议 1000，约 5s）。
 * 同时校准 X/Y/Z 三轴，Z 轴零偏供偏航角速度环使用。
 * 返回 1 完成（采样不再依赖 EXTI 标志，无超时概念）。
 */
uint8_t GyroBias_Calibrate(uint16_t samples);

/* 查询校准是否完成。 */
uint8_t GyroBias_IsReady(void);

/* 手动设置陀螺零偏（仅 X/Y 轴，单位：原始 LSB）。 */
void Set_Gyro_Bias(float bias_x, float bias_y);

/* 获取 Z 轴陀螺零偏（原始 LSB），调试用。 */
float Get_Gyro_Bias_Z(void);

/* 解锁时重置 PID 内部状态 + 低通滤波器，防止地面噪声污染导致解锁瞬态。 */
void Control_Arm_Reset(float current_gyro_pitch_dps, float current_gyro_roll_dps);

/*
 * Pitch/Roll 串级 PID 控制。
 * 校准已独立完成，此函数不再包含校准逻辑。
 */
void PID_Pitch_Roll_Combined(float actual_pitch, float actual_roll);

/*
 * 偏航角速度环 PID 控制（单环，500Hz）。
 * gyro_z_dps: Z 轴角速度（deg/s），已由调用方转换为 deg/s，内部再做零偏扣除。
 * 输出写入 pid_rate_yaw.output，由混控层加载 yaw 项。
 */
void PID_Yaw_Rate_Control(float gyro_z_dps);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
