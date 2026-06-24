#include "Altitude.h"

#include "MPU6050_Int.h"
#include "MPU6050.h"
#include "LED.h"

/*
 * 互补滤波高度融合：
 *
 *   加速度计 Z 轴(快200Hz) 二重积分 → 短时位置(精确但发散)
 *   BMP280 气压计  (慢20Hz)          → 长期高度(稳定但噪声)
 *                ↓ 互补滤波
 *           Alt_Fused (精确 + 稳定)
 *
 * 数学（20Hz 调用）：
 *   pos += vel * dt + Kp * (baro - pos)
 *   vel += acc * dt + Ki * (baro - pos)
 *
 * 其中 acc = (aacz - gravity_ref) * G_SCALE  (LSB → m/s²)
 */

/* 重力参考值 (LSB)，Altitude_Init() 静止采集 */
static float gravity_ref = 0.0f;

/* 就绪标志 */
static uint8_t s_ready = 0U;

/* =========================================================================
 * 可调参数 (20Hz 调用，每 50ms 一次)
 *   Kp: 气压计拉回强度，一次修正 Kp*error，建议 0.3~0.6
 *   Ki: 速度零偏修正，一次修正 Ki*error，建议 0.1~0.3
 * ========================================================================= */
#define ALT_KP  0.4f
#define ALT_KI  0.2f

/* LSB → m/s²: 9.80665 / 16384 (MPU6050 ±2g 灵敏度) */
#define G_SCALE  0.00059855f

/* =========================================================================
 * 融合输出
 * ========================================================================= */
float Alt_Fused    = 0.0f;
float Alt_Velocity = 0.0f;

/* =========================================================================
 * Altitude_Init — 5 秒静止采集重力参考
 * ========================================================================= */
void Altitude_Init(float grav_ref)
{
    if (grav_ref > 0.0f)
    {
        /* 使用 GyroBias_Calibrate 同步采集的重力参考，跳过自身校准 */
        gravity_ref = grav_ref;
    }
    else
    {
        /* 回退：自身 5 秒采集 */
        float sum = 0.0f;
        uint16_t i;
        uint16_t valid = 0U;
        for (i = 0U; i < 1000U; i++)
        {
            uint32_t timeout = 500000U;
            while (mpu_flag == 0U && timeout > 0U) { timeout--; }
            if (timeout == 0U) { break; }
            mpu_flag = 0U;
            mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
            MPU_Get_Accelerometer(&aacx, &aacy, &aacz);
            sum += (float)aacz;
            valid++;
        }
        gravity_ref = (valid > 0U) ? (sum / (float)valid) : 16384.0f;
    }
    Alt_Fused    = 0.0f;
    Alt_Velocity = 0.0f;
    s_ready      = 1U;
}

uint8_t Altitude_IsReady(void)
{
    return s_ready;
}

/* =========================================================================
 * Altitude_Update — 互补滤波融合
 *
 * 流程：
 *   1) aacz → 减去重力 → LSB→m/s² → 加速度
 *   2) 加速度积分 → 速度
 *   3) 速度积分 → 位置
 *   4) 气压计 P 校正 → 拉回位置漂移
 *   5) 气压计 I 校正 → 补偿速度零偏
 * ========================================================================= */
void Altitude_Update(short raw_acc_z, float baro_alt, float dt)
{
    float accel;
    float error;

    if (s_ready == 0U) { return; }
    if (dt <= 0.0f)     { return; }

    /* 1) 加速度：减去重力参考 → m/s² */
    accel = ((float)raw_acc_z - gravity_ref) * G_SCALE;

    /* 2) 速度积分 */
    Alt_Velocity += accel * dt;

    /* 3) 位置积分 */
    Alt_Fused += Alt_Velocity * dt;

    /* 4) 气压计互补校正（不乘 dt，每次直接修正 ALT_KP*error） */
    error = baro_alt - Alt_Fused;
    Alt_Fused    += ALT_KP * error;
    Alt_Velocity += ALT_KI * error;

    /* 钳位：相对地面高度不可为负 */
    if (Alt_Fused < 0.0f) { Alt_Fused = 0.0f; }
}
