#include "Control.h"
#include "Control_Task/Control_Task.h"  /* pid_task_flag / nrf_task_flag 等标志位 */
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

/* =========================================================================
 * 陀螺零偏（原始 LSB），仅 X/Y 轴。
 * Z 轴零偏由 IMU 互补滤波在线估计，不在此校准。
 * 上电后由 GyroBias_Calibrate() 采样计算，或由 Set_Gyro_Bias() 手动写入。
 * ========================================================================= */
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;

static uint8_t s_gyro_bias_ready = 0U;

/* =========================================================================
 * 控制回路周期：500Hz
 * ========================================================================= */
#define CONTROL_DT_S (0.002f)

/* =========================================================================
 * PID 对象
 * ========================================================================= */
PID_TypeDef pid_pitch;
PID_TypeDef pid_roll;
PID_TypeDef pid_yaw;

PID_TypeDef pid_rate_pitch;
PID_TypeDef pid_rate_roll;
PID_TypeDef pid_rate_yaw;

/* Pitch/Roll 串级对象 */
static PID_Cascade_t cascade_pitch;
static PID_Cascade_t cascade_roll;

/* 角速度低通：每轴一个独立实例，避免通道串扰 */
static LPF1_t gyro_pitch_lpf;
static LPF1_t gyro_roll_lpf;

/* =========================================================================
 * 内部辅助
 * ========================================================================= */

/* 将陀螺仪原始值转换为角速度（deg/s） */
static float GyroRawToDps(short raw, float bias)
{
	return ((float)raw - bias) / GYRO_SENS_2000DPS;
}

/* =========================================================================
 * GyroBias_Calibrate — 陀螺零偏校准（仅 X/Y 轴）
 *
 * Z 轴零偏由 IMU 互补滤波器在线估计，不在此校准。
 *
 * 上电后调用一次，飞行器必须保持静止。
 *
 * samples: 采样点数。DMP 输出 200Hz，1000 点 ≈ 5 秒。
 * 返回 1 成功，0 超时失败。
 *
 * 注意：调用前必须已完成 mpu_dmp_init() + Enroll_MPU6050_Register()
 * ========================================================================= */
uint8_t GyroBias_Calibrate(uint16_t samples)
{
	float acc_x = 0.0f;
	float acc_y = 0.0f;
	uint16_t i;

	if (samples == 0U)
	{
		samples = 1000U;
	}

	for (i = 0U; i < samples; i++)
	{
		uint32_t timeout = 500000U;
		while (mpu_flag == 0U && timeout > 0U)
		{
			timeout--;
		}

		if (timeout == 0U)
		{
			usart_printf(USART1, "Gyro calib TIMEOUT at %u/%u\r\n",
			             (unsigned int)i, (unsigned int)samples);
			return 0U;
		}

		mpu_flag = 0U;
		mpu_dmp_get_data(&Pitch, &Roll, &Yaw);
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

		acc_x += (float)gyrox;
		acc_y += (float)gyroy;
	}

	gyro_bias_x = acc_x / (float)samples;
	gyro_bias_y = acc_y / (float)samples;
	s_gyro_bias_ready = 1U;

	/*
	 * 打印校准结果 + 验证数据。
	 * 确认校准后静止角速度接近 0 即可注释掉，以后调试再解除。
	 */
	MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
	// usart_printf(USART1,
	//              "Gyro calib: bias(%.2f,%.2f)  dps(%.2f,%.2f) [%u samples]\r\n",
	//              (double)gyro_bias_x, (double)gyro_bias_y,
	//              (double)GyroRawToDps(gyrox, gyro_bias_x),
	//              (double)GyroRawToDps(gyroy, gyro_bias_y),
	//              (unsigned int)samples);

	return 1U;
}

/* 查询校准是否完成 */
uint8_t GyroBias_IsReady(void)
{
	return s_gyro_bias_ready;
}

/* 手动设置陀螺零偏（仅 X/Y 轴，单位：原始 LSB）。Z 轴零偏由 IMU 在线估计。 */
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
 *     Integral_max=100 → I_out 最大 ≈ 500（DShot 范围 ~25%），避免 I 项过主导
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

	/* ---- 积分分离阈值 ---- */
	PID_SetIntegralSeparation(&pid_pitch, 15.0f);
	PID_SetIntegralSeparation(&pid_roll,  15.0f);
	PID_SetIntegralSeparation(&pid_rate_pitch, 100.0f);
	PID_SetIntegralSeparation(&pid_rate_roll,  100.0f);

	/* ---- 串级：外环输出 → 内环目标 ---- */
	PID_Cascade_Init(&cascade_pitch, &pid_pitch, &pid_rate_pitch);
	PID_Cascade_Init(&cascade_roll,  &pid_roll,  &pid_rate_roll);

	/* ---- 陀螺角速度低通滤波器 ---- */
	LPF1_Init(&gyro_pitch_lpf, 0.45f, 0.0f);
	LPF1_Init(&gyro_roll_lpf,  0.45f, 0.0f);
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
	LPF1_Init(&gyro_pitch_lpf, 0.45f, current_gyro_pitch_dps);
	LPF1_Init(&gyro_roll_lpf,  0.45f, current_gyro_roll_dps);
}

/* =========================================================================
 * PID_Pitch_Roll_Combined — Pitch/Roll 串级 PID（500Hz）
 *
 * 调用前提：
 *   - GyroBias_Calibrate() 已完成
 *   - main loop 已清零 pid_task_flag
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

	/* 加载到电机混控 → DShot 输出 */
	Motor_Test();
}
