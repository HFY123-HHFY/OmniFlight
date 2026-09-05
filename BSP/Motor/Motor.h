#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "Dshot.h"
#include "KEY.h"
#include "Control.h"
#include "LED.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BSP/Motor — 电机混控模块
 *
 * 职责：
 * - 将遥控器油门 + PID 姿态修正量（pitch/roll/yaw）按 X 型混控矩阵分配到 4 路电机
 * - 混控反饱和：输出超出 DShot 范围时整体平移，保留姿态力矩
 * - 电机掉电时缓降油门，避免骤停
 *
 * 依赖：
 * - BSP/Dshot：DShot_Write + DSHOT_THROTTLE_MIN/MAX
 * - app/Control：pid_rate_pitch / pid_rate_roll / pid_rate_yaw（角速度环 PID 输出）
 * - app/Control：Alt_Throttle_Out（定高环油门偏差输出，±s_alt_cfg.out_max）
 * - app/Control：ALT_DEV_OUT_MAX（Key==3 预设基准油门）
 * - BSP/KEY：Key 状态（1=解锁 2=锁定停机 3=解锁+预设基准油门）
 */

/* 电机基础油门值（由遥控器油门摇杆提供，范围 DSHOT_THROTTLE_MIN ~ DSHOT_THROTTLE_MAX）。
 * Key==1 解锁时直接参与混控（base = speed_temp + 定高环偏差输出）。 */
extern uint16_t speed_temp;

/* 4 路电机最终 DShot 油门输出值 */
extern uint16_t Motor_Output[4];

/*
 * 电机混控测试函数。
 * 在控制任务中调用，根据按键触发：
 * - Key==1: 解锁，base = speed_temp + Alt_Throttle_Out + 角速度 PID 输出 → 混控 → DShot 发送
 * - Key==3: 解锁+预设基准，base = ALT_DEV_OUT_MAX + Alt_Throttle_Out
 *           + 角速度 PID 输出 → 混控 → DShot 发送
 * - Key==2: 缓降油门至最小值
 */
void Motor_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
