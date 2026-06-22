#include "IMU.h"

/*
 * IMU.c — 偏航角互补滤波融合
 *
 * ┌─ 启动阶段 ──────────────────────────────────────────┐
 * │ 上电静止 5 秒，采 2500 个 gyro_z 样本取平均 → 零偏  │
 * │ 首次磁力计数据直接设 yaw 起点，无需收敛等待          │
 * └─────────────────────────────────────────────────────┘
 *
 * ┌─ 运行阶段 (500Hz TIM1 ISR) ─────────────────────────┐
 * │ yaw += (gyro_z - bias) * dt      陀螺积分，扣零偏    │
 * └─────────────────────────────────────────────────────┘
 *
 * ┌─ 运行阶段 (50Hz 主循环，QMC 数据到达时) ────────────┐
 * │ error = mag - yaw                磁力计与融合值偏差   │
 * │ yaw  += Kp * error               P: 拉向磁力计       │
 * │ bias -= Ki * error               I: 温度漂移在线补偿  │
 * └─────────────────────────────────────────────────────┘
 *
 * Kp=0.15 @50Hz: 小误差 (<5°) 约 0.5 秒收敛
 * Ki=0.002@50Hz: 温度漂移在秒级被自动补偿
 */

/* ── 可调参数 ── */

#define IMU_BIAS_INIT_SAMPLES  2500U   /* 启动零偏采集样本数 (500Hz × 5秒) */
#define IMU_YAW_KP             0.15f   /* P 增益：磁力计校正强度             */
#define IMU_YAW_KI             0.002f  /* I 增益：零偏在线温度补偿           */

/* ── 内部状态 ── */

static float    s_yaw_fused;    /* 融合偏航角 [0, 360)                      */
static float    s_gyro_bias;    /* 陀螺 Z 轴零偏估计值 (deg/s)              */
static float    s_bias_sum;     /* 启动阶段 gyro_z 累加值                   */
static uint16_t s_bias_cnt;     /* 启动阶段已采集样本数                      */
static uint8_t  s_bias_ready;   /* 零偏就绪标志：1=完成，可开始融合          */
static uint8_t  s_yaw_seeded;   /* 起点初始化标志：1=已用 QMC 设过初值       */

/* ── API ── */

/*
 * IMU_Init — 上电调用一次，重置全部内部状态。
 * 调用后 IMU 进入启动阶段，需等待 IMU_IsReady() 返回 1。
 */
void IMU_Init(void)
{
	s_yaw_fused  = 0.0f;
	s_gyro_bias  = 0.0f;
	s_bias_sum   = 0.0f;
	s_bias_cnt   = 0U;
	s_bias_ready = 0U;
	s_yaw_seeded = 0U;
}

/*
 * IMU_Yaw_IntegrateGyro — TIM1 ISR 每 2ms (500Hz) 调用。
 * 启动阶段：采集 gyro_z 样本计算零偏均值，不积分。
 * 运行阶段：零偏补偿后累加角度增量。
 *
 *   gyro_z_dps : Z 轴角速度，已转为 deg/s（gyroz / 16.4）
 *   dt         : 积分步长 (s)，500Hz 时 = 0.002
 */
void IMU_Yaw_IntegrateGyro(float gyro_z_dps, float dt)
{
	if (!s_bias_ready)
	{
		/* ── 启动阶段：累加样本，满 2500 个取平均 ── */
		s_bias_sum += gyro_z_dps;
		s_bias_cnt++;

		if (s_bias_cnt >= IMU_BIAS_INIT_SAMPLES)
		{
			s_gyro_bias  = s_bias_sum / (float)s_bias_cnt;
			s_bias_ready = 1U;
		}
		return;   /* 零偏就绪前不积分 */
	}

	/* ── 运行阶段：零偏补偿积分 ── */
	s_yaw_fused += (gyro_z_dps - s_gyro_bias) * dt;

	/* 归一化到 [0, 360) */
	while (s_yaw_fused >= 360.0f) s_yaw_fused -= 360.0f;
	while (s_yaw_fused <   0.0f) s_yaw_fused += 360.0f;
}

/*
 * IMU_Yaw_CorrectMag — 主循环每收到新 QMC 数据时 (50Hz) 调用。
 * 首次调用直接用磁力计角度初始化起点，之后用 PI 校正。
 *
 *   mag_heading : QMC 航向角 (0~360 deg)，来自 QMC_Data()
 */
void IMU_Yaw_CorrectMag(float mag_heading)
{
	float error;

	if (!s_bias_ready) return;   /* 零偏未就绪，不校正 */

	if (!s_yaw_seeded)
	{
		/* 首次 QMC 数据直接设起点，零追赶延迟 */
		s_yaw_fused  = mag_heading;
		s_yaw_seeded = 1U;
		return;
	}

	/* 角度解绕：取最短路径 */
	error = mag_heading - s_yaw_fused;
	if      (error >  180.0f) error -= 360.0f;
	else if (error < -180.0f) error += 360.0f;

	/* PI 校正：P 拉向磁力计，I 学零偏温度漂移 */
	s_yaw_fused += IMU_YAW_KP * error;
	s_gyro_bias -= IMU_YAW_KI * error;

	/* 归一化 */
	while (s_yaw_fused >= 360.0f) s_yaw_fused -= 360.0f;
	while (s_yaw_fused <   0.0f) s_yaw_fused += 360.0f;
}

/* IMU_Get_Yaw — 获取当前融合偏航角 (0~360 deg)，任意时刻可调用。 */
float IMU_Get_Yaw(void)
{
	return s_yaw_fused;
}

/* IMU_Get_GyroBias — 获取当前零偏估计值 (deg/s)，调试/遥测用。 */
float IMU_Get_GyroBias(void)
{
	return s_gyro_bias;
}

/*
 * IMU_IsReady — 零偏采集是否完成。
 * 返回 0：启动中，融合未开始，IMU_Get_Yaw 返回 0。
 * 返回 1：零偏就绪，融合正常运行。
 * 用途：调试时确认启动阶段已完成；运行时可在 PID 前判断是否可用。
 */
uint8_t IMU_IsReady(void)
{
	return s_bias_ready;
}
