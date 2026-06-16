#include "Motor.h"

/*
 * BSP/Motor/Motor.c — 电机混控实现
 *
 * 混控矩阵（X 型四轴）：
 *   M1 = Base + Pitch + Roll
 *   M2 = Base - Pitch + Roll
 *   M3 = Base + Pitch - Roll
 *   M4 = Base - Pitch - Roll
 *
 * 反饱和策略：当任一电机输出超出 DShot 范围时，
 * 四路统一平移 shift，保留差分关系（姿态力矩），再做限幅。
 */

/* 电机基础油门值 - 由遥控器摇杆提供 */
uint16_t speed_temp = 0;

/* 4 路电机最终 DShot 油门输出 */
uint16_t Motor_Output[4] = {
    DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN,
    DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MIN
};

/* ---- 内部辅助函数 ------------------------------------------------ */

/*
 * 将浮点油门值限幅到 DShot 有效区间 [DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX]。
 */
static uint16_t Motor_DShotClamp(float val)
{
    if (val < (float)DSHOT_THROTTLE_MIN)
    {
        return DSHOT_THROTTLE_MIN;
    }
    if (val > (float)DSHOT_THROTTLE_MAX)
    {
        return DSHOT_THROTTLE_MAX;
    }
    return (uint16_t)val;
}

/*
 * 缓降电机油门到最小值。
 * 每次调用降低 step 个单位，用于掉电解锁时的平稳停机。
 */
static uint16_t Motor_RampDownToMin(uint16_t current, uint16_t step)
{
    if (current <= DSHOT_THROTTLE_MIN)
    {
        return DSHOT_THROTTLE_MIN;
    }

    if (current > (uint16_t)(DSHOT_THROTTLE_MIN + step))
    {
        return (uint16_t)(current - step);
    }

    return DSHOT_THROTTLE_MIN;
}

/*
 * 混控反饱和：
 * 1) 按 X 型混控矩阵计算四路理想输出
 * 2) 检测是否超出 DShot 有效范围
 * 3) 整体平移 shift，保留姿态差分关系
 * 4) 最终限幅输出
 *
 * base:  基础油门值（摇杆）
 * pitch: Pitch 轴 PID 修正量
 * roll:  Roll 轴 PID 修正量
 * out_m1~out_m4: 四路输出（调用后写入）
 */
static void Motor_MixWithDesaturation(float base, float pitch, float roll,
                                      uint16_t *out_m1, uint16_t *out_m2,
                                      uint16_t *out_m3, uint16_t *out_m4)
{
    float m1_raw, m2_raw, m3_raw, m4_raw;
    float max_raw, min_raw;
    float shift;

    /* 1) 混控矩阵 → 四路理想输出 */
    m1_raw = base + pitch + roll;
    m2_raw = base - pitch + roll;
    m3_raw = base + pitch - roll;
    m4_raw = base - pitch - roll;

    /* 2) 找四路极值 */
    max_raw = m1_raw;
    if (m2_raw > max_raw) { max_raw = m2_raw; }
    if (m3_raw > max_raw) { max_raw = m3_raw; }
    if (m4_raw > max_raw) { max_raw = m4_raw; }

    min_raw = m1_raw;
    if (m2_raw < min_raw) { min_raw = m2_raw; }
    if (m3_raw < min_raw) { min_raw = m3_raw; }
    if (m4_raw < min_raw) { min_raw = m4_raw; }

    /* 3) 计算统一平移量 shift
     *    目标：把四路一起搬回 [DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX]
     *    优先处理上限溢出，再处理下限溢出 */
    shift = 0.0f;
    if (max_raw > (float)DSHOT_THROTTLE_MAX)
    {
        shift = (float)DSHOT_THROTTLE_MAX - max_raw;
    }
    if ((min_raw + shift) < (float)DSHOT_THROTTLE_MIN)
    {
        shift += (float)DSHOT_THROTTLE_MIN - (min_raw + shift);
    }

    /* 4) 统一平移 + 最终限幅 */
    m1_raw += shift;
    m2_raw += shift;
    m3_raw += shift;
    m4_raw += shift;

    *out_m1 = Motor_DShotClamp(m1_raw);
    *out_m2 = Motor_DShotClamp(m2_raw);
    *out_m3 = Motor_DShotClamp(m3_raw);
    *out_m4 = Motor_DShotClamp(m4_raw);
}

/* ---- 公开接口 ---------------------------------------------------- */

/*
 * 电机混控测试函数。
 * 在主循环中周期性调用。
 *
 * Key == 1: 解锁运行
 *   - 读取 speed_temp（摇杆油门）+ 角速度 PID 输出
 *   - 油门越高，PID 补偿权重越大（高油门时姿态控制力被相对削弱）
 *   - 混控反饱和 → DShot 发送
 *
 * Key == 2: 掉电停机
 *   - 清零 PID 输出
 *   - 每 2 次调用缓降 ramp_step 个单位（避免骤停）
 *   - 油门降至最小值后停止
 */
void Motor_Test(void)
{
    static uint16_t m1 = DSHOT_THROTTLE_MIN;
    static uint16_t m2 = DSHOT_THROTTLE_MIN;
    static uint16_t m3 = DSHOT_THROTTLE_MIN;
    static uint16_t m4 = DSHOT_THROTTLE_MIN;

    if (Key == 1)
    {
        /*
         * 油门补偿：高油门时 PID 输出权重自动提升，
         * 防止姿态控制力被淹没在大油门输出中。
         */
        float throttle_ratio = ((float)speed_temp - (float)DSHOT_THROTTLE_MIN)
                             / ((float)DSHOT_THROTTLE_MAX - (float)DSHOT_THROTTLE_MIN);
        /* 补偿系数范围 1.0 ~ 1.8，可根据实际机型调整 */
        float pid_comp_scale = 1.0f + 0.8f * throttle_ratio;

        Motor_MixWithDesaturation((float)speed_temp,
                                  pid_rate_pitch.output * pid_comp_scale,
                                  pid_rate_roll.output * pid_comp_scale,
                                  &m1, &m2, &m3, &m4);

        Motor_Output[0] = m1;
        Motor_Output[1] = m2;
        Motor_Output[2] = m3;
        Motor_Output[3] = m4;

        DShot_Write(m1, m2, m3, m4);
        LED_Control(LED2, LED_LOW);
    }
    else if (Key == 2)
    {
        /* 清零 PID 输出，防止停机过程姿态修正干扰 */
        pid_rate_pitch.output = 0.0f;
        pid_rate_roll.output = 0.0f;

        const uint16_t ramp_step = 8U;
        static uint8_t ramp_div = 0U;

        ramp_div++;
        if (ramp_div >= 2U)
        {
            ramp_div = 0U;

            m1 = Motor_RampDownToMin(m1, ramp_step);
            m2 = Motor_RampDownToMin(m2, ramp_step);
            m3 = Motor_RampDownToMin(m3, ramp_step);
            m4 = Motor_RampDownToMin(m4, ramp_step);

            Motor_Output[0] = m1;
            Motor_Output[1] = m2;
            Motor_Output[2] = m3;
            Motor_Output[3] = m4;
        }

        DShot_Write(m1, m2, m3, m4);
        LED_Control(LED2, LED_HIGH);
    }
}
