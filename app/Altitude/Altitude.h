#ifndef __ALTITUDE_H
#define __ALTITUDE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 融合后的高度 (m) 与垂直速度 (m/s) */
extern float Alt_Fused;
extern float Alt_Velocity;

/*
 * 高度融合初始化：
 *   gravity_ref: aacz 重力参考值 (GyroBias_Calibrate 同步采集)
 *   > 0 时直接使用，跳过自身校准
 *   ==0 时回退到自身 5s 采集（兼容旧调用方式）
 */
void Altitude_Init(float gravity_ref);

/* 查询融合是否就绪 */
uint8_t Altitude_IsReady(void);

/*
 * 互补滤波融合更新：
 *   raw_acc_z : 加速度计 Z 轴原始值 (LSB)
 *   baro_alt  : 气压计当前高度 (m)
 *   dt        : 距上次调用的时间 (s)，建议 0.005 (200Hz)
 *
 * 调用频率: 200Hz（mpu_flag 消费块中）
 */
void Altitude_Update(short raw_acc_z, float baro_alt, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __ALTITUDE_H */
