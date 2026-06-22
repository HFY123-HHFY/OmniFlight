#include "IMU.h"

/*
 * IMU.c — 偏航角互补滤波融合
 *
 * 启动阶段 (LED3 亮):
 *   上电静止 5 秒，每 2ms 采一个 gyro_z 样本，2500 个取平均 → 初始零偏
 *   首次磁力计数据直接设为 yaw 起点
 *
 * 运行阶段 (LED3 灭):
 *   yaw += (gyro_z - bias) * dt    [500Hz]  零偏补偿积分
 *   error = mag - yaw              [ 50Hz]  磁力计误差
 *   yaw  += Kp * error             [ 50Hz]  P: 拉向磁力计
 *   bias -= Ki * error             [ 50Hz]  I: 温度漂移在线微调
 */

/* ── 可调参数 ── */

#define IMU_BIAS_INIT_SAMPLES  2500U   /* 启动采样数 (500Hz × 5秒) */
#define IMU_YAW_KP             0.15f   /* P 增益：磁力计校正强度 */
#define IMU_YAW_KI             0.002f  /* I 增益：零偏在线微调 */

/* ── 内部状态 ── */

static float    s_yaw_fused   = 0.0f;
static float    s_gyro_bias   = 0.0f;
static float    s_bias_sum    = 0.0f;
static uint16_t s_bias_cnt    = 0U;
static uint8_t  s_bias_ready  = 0U;
static uint8_t  s_yaw_seeded  = 0U;

/* ── 调试 printf（校准阶段临时使用，不污染头文件依赖）── */
extern void usart_printf(void *uart, const char *fmt, ...);
#define IMU_DBG_USART  ((void *)0x40011000UL)

/* ── API ── */

void IMU_Init(void)
{
	s_yaw_fused  = 0.0f;
	s_gyro_bias  = 0.0f;
	s_bias_sum   = 0.0f;
	s_bias_cnt   = 0U;
	s_bias_ready = 0U;
	s_yaw_seeded = 0U;
}

void IMU_Yaw_IntegrateGyro(float gyro_z_dps, float dt)
{
	if (!s_bias_ready)
	{
		s_bias_sum += gyro_z_dps;
		s_bias_cnt++;

		if (s_bias_cnt >= IMU_BIAS_INIT_SAMPLES)
		{
			s_gyro_bias  = s_bias_sum / (float)s_bias_cnt;
			s_bias_ready = 1U;

			/* 打印捕获到的零偏值 */
			usart_printf(IMU_DBG_USART,
				"\r\n[IMU] bias=%.3f dps (%u samples)\r\n",
				(double)s_gyro_bias, (unsigned int)s_bias_cnt);
		}
		return;
	}

	s_yaw_fused += (gyro_z_dps - s_gyro_bias) * dt;

	while (s_yaw_fused >= 360.0f) s_yaw_fused -= 360.0f;
	while (s_yaw_fused <   0.0f) s_yaw_fused += 360.0f;
}

void IMU_Yaw_CorrectMag(float mag_heading)
{
	float error;

	if (!s_bias_ready) return;

	/* 首次 QMC 数据直接设起点 */
	if (!s_yaw_seeded)
	{
		s_yaw_fused  = mag_heading;
		s_yaw_seeded = 1U;
		return;
	}

	error = mag_heading - s_yaw_fused;

	if      (error >  180.0f) error -= 360.0f;
	else if (error < -180.0f) error += 360.0f;

	s_yaw_fused += IMU_YAW_KP * error;
	s_gyro_bias -= IMU_YAW_KI * error;

	while (s_yaw_fused >= 360.0f) s_yaw_fused -= 360.0f;
	while (s_yaw_fused <   0.0f) s_yaw_fused += 360.0f;
}

float IMU_Get_Yaw(void)
{
	return s_yaw_fused;
}

float IMU_Get_GyroBias(void)
{
	return s_gyro_bias;
}

uint8_t IMU_IsReady(void)
{
	return s_bias_ready;
}
