#include "Motor.h"

/*
 * BSP/Motor/Motor.c — 电机混控实现
 *
 * 混控矩阵（X 型四轴）：
 *   M1 = Base + Pitch + Roll + Yaw*YAW_DIR
 *   M2 = Base - Pitch + Roll - Yaw*YAW_DIR
 *   M3 = Base + Pitch - Roll + Yaw*YAW_DIR
 *   M4 = Base - Pitch - Roll - Yaw*YAW_DIR
 *
 * 偏航力矩由对角电机反扭矩差产生（M1/M3 对角同向，M2/M4 对角同向）。
 * 台架测试：解锁后手持机身绕 Z 轴转动，若感受到的阻力矩方向相反，
 * 翻转 MOTOR_YAW_DIR 符号即可。
 *
 * 反饱和策略：当任一电机输出超出 DShot 范围时，
 * 四路统一平移 shift，保留差分关系（姿态力矩），再做限幅。
 */

/* 偏航混控方向：+1 或 -1，台架验证后确定 */
#define MOTOR_YAW_DIR (1.0f)

/* 电机基础油门值 - 由遥控器油门摇杆提供（0~250 → 48~2047）。
 * Key==1 解锁：直接作混控 base 的一部分（base = speed_temp + Alt_Throttle_Out）；
 * Key==3 解锁+预设：不参与（base = ALT_DEV_OUT_MAX + Alt_Throttle_Out）。 */
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
 * base:  基础油门值（Key==1 = speed_temp + Alt_Throttle_Out / Key==3 = ALT_DEV_OUT_MAX + Alt_Throttle_Out）
 * pitch: Pitch 轴 PID 修正量
 * roll:  Roll 轴 PID 修正量
 * yaw:   Yaw 轴 PID 修正量（方向由 MOTOR_YAW_DIR 决定）
 * out_m1~out_m4: 四路输出（调用后写入）
 */
static void Motor_MixWithDesaturation(float base, float pitch, float roll, float yaw,
                                      uint16_t *out_m1, uint16_t *out_m2,
                                      uint16_t *out_m3, uint16_t *out_m4)
{
    float m1_raw, m2_raw, m3_raw, m4_raw;
    float max_raw, min_raw;
    float shift;
    float yaw_term = yaw * MOTOR_YAW_DIR;

    /* 1) 混控矩阵 → 四路理想输出 */
    m1_raw = base + pitch + roll + yaw_term;
    m2_raw = base - pitch + roll - yaw_term;
    m3_raw = base - pitch - roll + yaw_term;
    m4_raw = base + pitch - roll - yaw_term;

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
 * 在控制任务中周期性调用。
 *
 * Key == 1: 解锁飞行
 *   - base = speed_temp + Alt_Throttle_Out：
 *     油门摇杆直接打底给油（推多少给多少），Alt_Throttle_Out 为
 *     Altitude_Stick_Input 定高环 P+I 偏差修正，叠加在摇杆基准上
 *
 * Key == 3: 解锁 + 预设基准油门
 *   - base = ALT_DEV_OUT_MAX + Alt_Throttle_Out：
 *     预设基准（约悬停油门）出大力，Alt_Throttle_Out 为定高环 P+I
 *     偏差修正，PID 只出偏差（不累）；speed_temp 不参与；
 *   Altitude_Stick_Input 回中摇杆经定高环控高度（Key==1/3 均生效）。
 *
 * Key == 1/3 共用：
 *   - 三轴角速度 PID 输出作姿态修正
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

    if ((Key == 1) || (Key == 3))
    {
        /*
         * base 按 Key 状态决定（由遥控器切换）：
         *   Key==1 解锁      = Alt_Throttle_Out
         *     —— 油门摇杆直接打底给油，Altitude_Stick_Input 定高环在摇杆基准上出偏差修正
         *   Key==3 解锁+预设 = ALT_DEV_OUT_MAX + Alt_Throttle_Out
         *     —— 预设基准油门出大力，PID 只出偏差修正（speed_temp 不参与）
         *
         * 油门补偿：高油门时 PID 输出权重自动提升，
         * 防止姿态控制力被淹没在大油门输出中。
         */
        // float base = (float)speed_temp + Alt_Throttle_Out;
        float base = Alt_Throttle_Out;
        if (Key == 3)
        {
            base = ALT_DEV_OUT_MAX + Alt_Throttle_Out;
        }
        float throttle_ratio = (base - (float)DSHOT_THROTTLE_MIN)
                             / ((float)DSHOT_THROTTLE_MAX - (float)DSHOT_THROTTLE_MIN);
        if (throttle_ratio < 0.0f)
        {
            throttle_ratio = 0.0f;  /* Key==1 起步 base 可能低于 DShot 下限 */
        }
        /* 补偿系数范围 1.0 ~ 1.8，可根据实际机型调整 */
        float pid_comp_scale = 1.0f + 0.8f * throttle_ratio;

        Motor_MixWithDesaturation(base,
                                  pid_rate_pitch.output * pid_comp_scale,
                                  pid_rate_roll.output * pid_comp_scale,
                                  pid_rate_yaw.output * pid_comp_scale,
                                  &m1, &m2, &m3, &m4);

        Motor_Output[0] = m1;
        Motor_Output[1] = m2;
        Motor_Output[2] = m3;
        Motor_Output[3] = m4;

        DShot_Write(m1, m2, m3, m4);
    }
    else if (Key == 2)
    {
        /* 清零 PID 输出，防止停机过程姿态修正干扰 */
        pid_rate_pitch.output = 0.0f;
        pid_rate_roll.output = 0.0f;
        pid_rate_yaw.output   = 0.0f;
        Alt_Throttle_Out      = 0.0f;

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
    }
}
