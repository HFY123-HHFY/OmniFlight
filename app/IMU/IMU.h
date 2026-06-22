#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU.h — 姿态融合模块
 *
 * 互补滤波器融合陀螺仪角速度 + 磁力计航向：
 *   yaw = ∫ gyro_z·dt          ← 高频：500Hz 陀螺积分，响应快
 *        + Kp × (mag - yaw)     ← 低频：磁力计缓慢拉回，消除漂移
 *
 * 磁力计是"指南针"，告诉你绝对方向，但转快了跟不上、易受干扰。
 * 陀螺仪是"转速表"，告诉你转了多少，但积分久了会漂。
 * 互补滤波取两者之长。
 */

/* 初始化融合状态（上电时调用一次）。 */
void IMU_Init(void);

/*
 * 陀螺积分 — TIM1 ISR 每 500Hz 调用。
 *   gyro_z_dps : Z 轴角速度 (deg/s)，已应用安装方向变换
 *   dt         : 积分步长 (s)，500Hz = 0.002
 */
void IMU_Yaw_IntegrateGyro(float gyro_z_dps, float dt);

/*
 * 磁力计校正 — 主循环收到新 QMC 数据时调用（~50Hz）。
 *   mag_heading : QMC 航向角 (0~360 deg)
 */
void IMU_Yaw_CorrectMag(float mag_heading);

/* 获取融合后的偏航角 (0~360 deg)。 */
float IMU_Get_Yaw(void);

/* 获取自动估计的陀螺 Z 轴零偏 (deg/s)。调试用。 */
float IMU_Get_GyroBias(void);

/* 返回零偏初始化是否完成（完成后才能用 IMU_Get_Yaw）。 */
uint8_t IMU_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
