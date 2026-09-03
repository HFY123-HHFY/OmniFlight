#include "Control.h"
#include "Delay.h"
#include "LED.h"
#include "MPU6050.h"                    /* MPU_Get_Gyroscope */
#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "KEY.h"

/* =========================================================================
 * 目标姿态
 * ========================================================================= */
float Target_Pitch = 0.0f;
float Target_Roll  = 0.0f;
float Target_Yaw   = 0.0f;

/* 目标偏航角速度（deg/s）：当前恒 0 消除自旋，后续由遥控器写入 */
float Target_Yaw_Rate = 0.0f;

/* =========================================================================
 * 陀螺零偏（原始 LSB），X/Y/Z 三轴。
 * 上电后由 GyroBias_Calibrate() 采样计算，或由 Set_Gyro_Bias() 手动写入。
 * Z 轴零偏供偏航角速度环去偏使用。
 * ========================================================================= */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static uint8_t s_gyro_bias_ready = 0U;

/* =========================================================================
 * 控制回路周期：500Hz
 * ========================================================================= */
#define CONTROL_DT_S (0.002f)

/* =========================================================================
 * 偏航环滤波与死区参数（500Hz）
 *
 * 台架测试发现：静止时 gyro_z 偶发单拍跳变（振动/总线毛刺），
 * 且转动测试的积分残留会持续给输出，电机"卡卡"抖几下不停。
 * 组合对策：低通加强 + 角速度死区 + 输出死区 + 死区内积分泄放。
 * ========================================================================= */
#define YAW_GYRO_LPF_ALPHA    (0.20f) /* 偏航低通系数，截止约 20Hz，抗振动毛刺 */
#define YAW_RATE_DEADBAND_DPS (3.0f)  /* 角速度死区：低通后 |rate|≤3dps 视为静止 */
#define YAW_OUTPUT_DEADBAND   (10.0f) /* 输出死区：|yaw_out|≤10 清零，防电机抖动 */
#define YAW_INTEGRAL_DECAY    (0.99f) /* 死区内每拍积分泄放系数，防转动后残留输出 */

/* =========================================================================
 * PID 对象
 * ========================================================================= */
PID_TypeDef pid_pitch;
PID_TypeDef pid_roll;

PID_TypeDef pid_rate_pitch;
PID_TypeDef pid_rate_roll;

/* 高度 alt 对象*/
PID_TypeDef pid_alt;

/* 偏航角速度环对象 */
PID_TypeDef pid_rate_yaw;

/* Pitch/Roll 串级对象 */
static PID_Cascade_t cascade_pitch;
static PID_Cascade_t cascade_roll;

/* 角速度低通：每轴一个独立实例，避免通道串扰 */
static LPF1_t gyro_pitch_lpf;
static LPF1_t gyro_roll_lpf;
static LPF1_t gyro_yaw_lpf;

/* =========================================================================
 * 内部辅助
 * ========================================================================= */

/* 将陀螺仪原始值转换为角速度（deg/s） */
static float GyroRawToDps(short raw, float bias)
{
	return ((float)raw - bias) / GYRO_SENS_2000DPS;
}

/* =========================================================================
 * GyroBias_Calibrate — 陀螺零偏校准（X/Y/Z 三轴）
 *
 * Z 轴零偏同样在此校准，供偏航角速度环去偏使用。
 *
 * 上电后调用一次，飞行器必须保持静止。
 *
 * samples: 采样点数。固定 5ms 采样节拍 (200Hz)，1000 点 ≈ 5 秒。
 * 返回 1 完成（采样不再依赖 EXTI 标志，无超时概念）。
 *
 * 注意：调用前必须已完成 mpu_dmp_init()。本函数在调度器启动前调用，
 *       采样节拍用 Delay_ms（DWT 忙等），不与 RTOS tick 冲突。
 * ========================================================================= */
uint8_t GyroBias_Calibrate(uint16_t samples)
{
	float gyro_sum_x = 0.0f;
	float gyro_sum_y = 0.0f;
	float gyro_sum_z = 0.0f;
	uint16_t i;
	LED_Level_t led3Level = LED_HIGH;

	if (samples == 0U)
	{
		samples = 1000U;
	}

	for (i = 0U; i < samples; i++)
	{
		Delay_ms(5U);   /* 固定 5ms 采样节拍 = 200Hz，与 DMP 输出率一致 */
		if (((i + 1U) % 100U) == 0U)
		{
			led3Level = (led3Level == LED_HIGH) ? LED_LOW : LED_HIGH;
			LED_Control(LED3, led3Level);
		}

		mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

		gyro_sum_x += (float)gyrox;
		gyro_sum_y += (float)gyroy;
		gyro_sum_z += (float)gyroz;
	}

	gyro_bias_x = gyro_sum_x / (float)samples;
	gyro_bias_y = gyro_sum_y / (float)samples;
	gyro_bias_z = gyro_sum_z / (float)samples;
	s_gyro_bias_ready = 1U;

	/*
	 * 打印校准结果 + 验证数据。
	 * 确认校准后静止角速度接近 0 即可注释掉，以后调试再解除。
	 */
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	// usart_printf(USART1,
	//              "Gyro calib: bias(%.2f,%.2f,%.2f)  dps(%.2f,%.2f,%.2f) [%u samples]\r\n",
	//              (double)gyro_bias_x, (double)gyro_bias_y, (double)gyro_bias_z,
	//              (double)GyroRawToDps(gyrox, gyro_bias_x),
	//              (double)GyroRawToDps(gyroy, gyro_bias_y),
	//              (double)GyroRawToDps(gyroz, gyro_bias_z),
	//              (unsigned int)samples);

	return 1U;
}

/* 获取 Z 轴陀螺零偏（原始 LSB），调试用。 */
float Get_Gyro_Bias_Z(void)
{
	return gyro_bias_z;
}

/* 查询校准是否完成 */
uint8_t GyroBias_IsReady(void)
{
	return s_gyro_bias_ready;
}

/* 手动设置陀螺零偏（仅 X/Y 轴，单位：原始 LSB）。 */
void Set_Gyro_Bias(float bias_x, float bias_y)
{
	gyro_bias_x = bias_x;
	gyro_bias_y = bias_y;
	s_gyro_bias_ready = 1U;
}

/* =========================================================================
 * PID_Contorl_Init — PID 初始化
 *
 * 限幅设计（基于 DShot300: 48~2047，dt=0.002s）：
 *
 *   外环（角度→角速度目标）：
 *     Out_max = 400    → 最大角速度指令 ±400°/s
 *     Integral_max=700 → I_out 最大=ki×700，被 Out_max 钳位，保证 I 可全额贡献
 *
 *   内环（角速度→电机输出）：
 *     Out_max = 2047   → 对称输出，混控层再做 DShot 区间限幅
 *     Integral_max=100 → I_out 最大 = ki×100（当前 ki=0.015 → ≈1.5），
 *                        调大 ki 时注意该上限（如 ki=2.5 → 250）
 *
 *   角速度低通：
 *     alpha = 0.45     → 截止频率 ~36Hz @ 500Hz 采样，抑制电机高频振动
 * ========================================================================= */
void PID_Contorl_Init(void)
{
	/* ---- 外环 PID：角度 → 角速度目标 ---- */
	PID_Init(&pid_pitch);
	PID_Init(&pid_roll);
	PID_Init_WithLimit(&pid_pitch, 700.0f, 400.0f);
	PID_Init_WithLimit(&pid_roll,  700.0f, 400.0f);

	/* ---- 内环 PID：角速度 → 电机输出 ---- */
	PID_Init(&pid_rate_pitch);
	PID_Init(&pid_rate_roll);
	PID_Init_WithLimit(&pid_rate_pitch, 100.0f, MOTOR_MIX_LIMIT);
	PID_Init_WithLimit(&pid_rate_roll,  100.0f, MOTOR_MIX_LIMIT);

	/* ---- 偏航角速度环 PID：gyro_z → 电机 yaw 项 ---- */
	/* 偏航力矩靠螺旋桨反扭矩差，权限远弱于俯仰/滚转，输出限幅给小值防止抢油门 */
	PID_Init(&pid_rate_yaw);
	PID_Init_WithLimit(&pid_rate_yaw, 50.0f, 500.0f);
	PID_SetDeadband(&pid_rate_yaw, YAW_RATE_DEADBAND_DPS); /* 静止微噪声按 0 处理 */

	/* ---- 积分分离阈值 ---- */
	PID_SetIntegralSeparation(&pid_pitch, 15.0f);
	PID_SetIntegralSeparation(&pid_roll,  15.0f);
	PID_SetIntegralSeparation(&pid_rate_pitch, 100.0f);
	PID_SetIntegralSeparation(&pid_rate_roll,  100.0f);
	PID_SetIntegralSeparation(&pid_rate_yaw,   30.0f); /* 转动测试时基本不积分，防残留 */

	/* ---- 串级：外环输出 → 内环目标 ---- */
	PID_Cascade_Init(&cascade_pitch, &pid_pitch, &pid_rate_pitch);
	PID_Cascade_Init(&cascade_roll,  &pid_roll,  &pid_rate_roll);

	/* ---- 陀螺角速度低通滤波器 ---- */
	LPF1_Init(&gyro_pitch_lpf, 0.45f, 0.0f);
	LPF1_Init(&gyro_roll_lpf,  0.45f, 0.0f);
	LPF1_Init(&gyro_yaw_lpf,   YAW_GYRO_LPF_ALPHA, 0.0f); /* 偏航轴加强滤波，抗振动毛刺 */
}

/* =========================================================================
 * Control_Arm_Reset
 *
 * 解锁时重置 PID内部状态+低通滤波器，防止地面噪声污染。
 * ========================================================================= */
void Control_Arm_Reset(float current_gyro_pitch_dps, float current_gyro_roll_dps)
{
	PID_Reset(&pid_pitch);
	PID_Reset(&pid_roll);
	PID_Reset(&pid_rate_pitch);
	PID_Reset(&pid_rate_roll);
	PID_Reset(&pid_rate_yaw);
	LPF1_Init(&gyro_pitch_lpf, 0.45f, current_gyro_pitch_dps);
	LPF1_Init(&gyro_roll_lpf,  0.45f, current_gyro_roll_dps);
	LPF1_Init(&gyro_yaw_lpf,   YAW_GYRO_LPF_ALPHA, GyroRawToDps(gyroz, gyro_bias_z));
}

/* =========================================================================
 * PID_Pitch_Roll_Combined — Pitch/Roll 串级 PID（500Hz）
 *
 * 调用前提：
 *   - GyroBias_Calibrate() 已完成
 *   - 由 ControlTask 每 500Hz 调用（dt = 0.002s）
 *
 * 数据流：
 *   gyrox/gyroy(原始LSB) → 去偏 → deg/s → 低通 → 内环PID → Motor_Test()
 *   Pitch/Roll(DMP角度)  → 外环PID → 角速度目标 ─┘
 * ========================================================================= */
void PID_Pitch_Roll_Combined(float actual_pitch, float actual_roll)
{
	static uint8_t last_key = 0U;

	/* 解锁边沿检测：Key 0->1 时重置 PID状态 */
	if (Key == 1 && last_key != 1)
	{
		Control_Arm_Reset(GyroRawToDps(gyroy, gyro_bias_y),
		                  GyroRawToDps(gyrox, gyro_bias_x));
	}
	last_key = Key;

	float pitch_rate_out = 0.0f;
	float roll_rate_out  = 0.0f;
	float gyro_pitch_dps = 0.0f;
	float gyro_roll_dps  = 0.0f;

	/* 角速度反馈：原始值 LSB → 去偏 → deg/s */
	gyro_roll_dps  = GyroRawToDps(gyrox, gyro_bias_x);
	gyro_pitch_dps = GyroRawToDps(gyroy, gyro_bias_y);

	/* 每轴独立低通，抑制高频振动 */
	gyro_roll_dps  = LPF1_Update(&gyro_roll_lpf,  gyro_roll_dps);
	gyro_pitch_dps = LPF1_Update(&gyro_pitch_lpf, gyro_pitch_dps);

	/* 外环目标角度（来自遥控/导航） */
	PID_SetTarget(&pid_pitch, Target_Pitch);
	PID_SetTarget(&pid_roll,  Target_Roll);

	/* 串级计算：角度外环 → 角速度内环 → 电机输出 */
	pitch_rate_out = PID_Cascade_Calc(&cascade_pitch,
	                                   actual_pitch, gyro_pitch_dps,
	                                   CONTROL_DT_S, CONTROL_DT_S);
	roll_rate_out  = PID_Cascade_Calc(&cascade_roll,
	                                   actual_roll, gyro_roll_dps,
	                                   CONTROL_DT_S, CONTROL_DT_S);

	/* 保留到 PID 对象，方便调试/遥测 */
	pid_rate_pitch.output = pitch_rate_out;
	pid_rate_roll.output  = roll_rate_out;
}

/* =========================================================================
 * PID_Yaw_Rate_Control — 偏航角速度环 PID（单环，500Hz）
 *
 * 数据流：
 *   gyroz(原始LSB) → 去偏(Z轴零偏) → deg/s → 低通 → PID → pid_rate_yaw.output
 *
 * 抗抖设计（台架实测：静止时偶发单拍跳变 + 转动测试积分残留导致电机卡顿）：
 *   1) 角速度死区（PID 库 deadband）：低通后 |rate|≤3dps 视为 0
 *   2) 输出死区：|yaw_out|≤10 直接清零，电机完全静止
 *   3) 积分泄放：输出在死区内时每拍 error_sum×0.99，防止积分残留持续给输出
 *
 * 混控调用（Motor_Test）由 ControlTask 统一发起，保证三轴 PID 全部算完后再混控。
 * ========================================================================= */
void PID_Yaw_Rate_Control(float gyro_z_dps)
{
	float gyro_yaw_dps;

	/* 角速度反馈：去偏 → 低通，抑制高频振动 */
	gyro_yaw_dps = gyro_z_dps - (gyro_bias_z / GYRO_SENS_2000DPS);
	gyro_yaw_dps = LPF1_Update(&gyro_yaw_lpf, gyro_yaw_dps);

	/* 单环计算：目标角速度（当前恒 0 消除自旋）→ 电机 yaw 项输出 */
	PID_SetTarget(&pid_rate_yaw, Target_Yaw_Rate);
	pid_rate_yaw.output = PID_CalcDt(&pid_rate_yaw, gyro_yaw_dps, CONTROL_DT_S);

	/* 输出死区 + 积分泄放 */
	if ((pid_rate_yaw.output < YAW_OUTPUT_DEADBAND) &&
	    (pid_rate_yaw.output > -YAW_OUTPUT_DEADBAND))
	{
		pid_rate_yaw.output    = 0.0f;
		pid_rate_yaw.error_sum *= YAW_INTEGRAL_DECAY;
	}
}
