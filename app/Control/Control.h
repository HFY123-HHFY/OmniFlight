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

/* 外环 PID：角度。 */
extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;

/* 内环 PID：角速度。 */
extern PID_TypeDef pid_rate_pitch;
extern PID_TypeDef pid_rate_roll;

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
 * samples        : 采样点数（建议 1000，约 5s）
 * gravity_ref_out: 输出重力参考值 (aacz 均值)，可为 NULL
 * 返回 1 成功，0 超时失败。
 */
uint8_t GyroBias_Calibrate(uint16_t samples, float *gravity_ref_out);

/* 查询校准是否完成。 */
uint8_t GyroBias_IsReady(void);

/* 手动设置陀螺零偏（仅 X/Y 轴，单位：原始 LSB）。Z 轴零偏由 IMU 在线估计。 */
void Set_Gyro_Bias(float bias_x, float bias_y);

/* 解锁时重置 PID 内部状态 + 低通滤波器，防止地面噪声污染导致解锁瞬态。 */
void Control_Arm_Reset(float current_gyro_pitch_dps, float current_gyro_roll_dps);

/*
 * Pitch/Roll 串级 PID 控制。
 * 校准已独立完成，此函数不再包含校准逻辑。
 */
void PID_Pitch_Roll_Combined(float actual_pitch, float actual_roll);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
