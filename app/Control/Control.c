#include "Control.h"
#include "Delay.h"
#include "LED.h"
#include "MPU6050.h"                    /* MPU_Get_Gyroscope */
#include "My_Usart/My_Usart.h"          /* usart_printf */
#include "KEY.h"
#include "MTF02P.h"                     /* mtf02p_data / MTF02P_IsRangeValid（定高环） */
#include "NRF24L01.h"                   /* Altitude_Stick_Input 摇杆（定高环） */

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
 * 定高环（100Hz，MTF02P ToF 距离；Key==1 解锁 / Key==3 解锁+预设基准时生效）
 *
 * 结构：Altitude_Stick_Input → 速率指令积分进 Alt_Target_M（回中冻结 = 保持当前高度）
 *       高度外环（纯 P）→ 爬升速率目标（+ 摇杆前馈，限幅）
 *       速率内环（P+I）→ 油门偏差输出 Alt_Throttle_Out（±out_max）
 *   混控 base（Motor.c）：
 *     Key==1 解锁      = speed_temp + Alt_Throttle_Out（油门摇杆打底，Altitude_Stick_Input 定高环出偏差）
 *     Key==3 解锁+预设 = ALT_DEV_OUT_MAX + Alt_Throttle_Out（预设基准出大力，
 *                       PID 只出偏差修正）
 *     Key==2 锁定      = 停机（本环不被调用，Motor_Test 缓降并清零输出）
 *
 * PID 增益与 Pitch/Roll/偏航环一致，在 main.c 用 Set_PID 调参；
 * 定高环专属参数（节拍/死区/限幅/抗扰）收在 Alt_Cfg_t 结构体
 * （类型见 Control.h，默认值由 Alt_Config_Init 加载）。
 * ========================================================================= */

Alt_Cfg_t s_alt_cfg;

/* 定高环默认配置。唯一宏 ALT_DEV_OUT_MAX（预设基准油门）在 Control.h。 */
static void Alt_Config_Init(void)
{
    s_alt_cfg.loop_div             = 5U;
    s_alt_cfg.loop_dt_s            = 0.01f;
    s_alt_cfg.rc_max_rate          = 1.0f;
    s_alt_cfg.rc_deadband          = 5;
    s_alt_cfg.pos_deadband_m       = 0.03f;
    s_alt_cfg.rate_target_max      = 1.5f;
    s_alt_cfg.i_max                = 8.0f;   /* ki=150 时 I_out 上限 ±1200 = out_max */
    s_alt_cfg.out_max              = 2.0f * ALT_DEV_OUT_MAX; /* 偏差窗口 = 2×预设基准，I 学悬停差额够用 */
    s_alt_cfg.rate_deadband_mps    = 0.10f;
    s_alt_cfg.rate_sep_mps         = 1.0f;
    s_alt_cfg.rate_lpf_alpha       = 0.30f;
    s_alt_cfg.dist_jump_m          = 0.5f;
    s_alt_cfg.invalid_reanchor_cnt = 50U;
    s_alt_cfg.ground_dist_m        = 0.02f; /* 实测地面恒读 20mm（ToF 量程下限） */
    /* 目标下限 = 地面：解锁后 t 停在地面值（摇杆不动），向上推才升高，
     * 向下推到底降到 0.02 = 落地（与地面死区一致） */
    s_alt_cfg.target_min_m         = 0.02f;
    s_alt_cfg.target_max_m         = 4.0f;
}

/* =========================================================================
 * 定点环（100Hz，MTF02P 光流；Key==1 解锁 / Key==3 解锁+预设基准时生效）
 *
 * 结构：Position_X/Y_Stick_Input 摇杆 → 速度指令（回中=0=悬停定点，推杆=移动）
 *       MTF02P 光流 × 高度 → 实测机体速度（低通 + 毛刺保护）
 *       速度环 pid_pos_x/y（P+I）→ 倾角指令（±out_max_deg）
 *       倾角指令经 dir_x/dir_y 方向修正写入 Target_Pitch（横向）/Target_Roll（纵向），
 *       复用 Pitch/Roll 串级外环（PID_Pitch_Roll_Combined）
 *
 * 方向约定（用户实测，机头朝上方位）：
 *   flow_x 左→右为正 / flow_y 机头→机尾为正；
 *   Position_X_Stick_Input +100 = 向右（flow_x 正）
 *   Position_Y_Stick_Input +100 = 向前（机尾→机头，flow_y 负）
 *
 * PID 增益与 Pitch/Roll/偏航/定高环一致，在 main.c 用 Set_PID 调参；
 * 定点环专属参数（节拍/高度窗/死区/限幅/抗扰/方向）收在 Pos_Cfg_t 结构体
 * （类型见 Control.h，默认值由 Pos_Config_Init 加载）。
 * ========================================================================= */

Pos_Cfg_t s_pos_cfg;

/* 定点环默认配置 */
static void Pos_Config_Init(void)
{
    s_pos_cfg.loop_div             = 5U;      /* 100Hz，与定高环一致 */
    s_pos_cfg.loop_dt_s            = 0.01f;
    s_pos_cfg.height_min_m         = 0.1f;    /* 低空飞行/手持测试为主，放宽到 0.1m（贴地仍由 ground_dist 归零保护） */
    s_pos_cfg.height_max_m         = 4.0f;    /* 与定高环目标上限一致 */
    s_pos_cfg.rc_max_mps           = 1.0f;    /* 满杆 ±1m/s */
    s_pos_cfg.rc_deadband          = 5;
    s_pos_cfg.vel_lpf_alpha        = 0.30f;   /* 光流单帧噪声大，与定高环速率低通同档 */
    s_pos_cfg.vel_jump_mps         = 2.0f;    /* 单拍速度跳变毛刺阈值 */
    s_pos_cfg.quality_threshold    = 40U;     /* flow_quality 低于此值纹理不可信 */
    s_pos_cfg.vel_deadband_mps     = 0.05f;
    s_pos_cfg.vel_sep_mps          = 0.8f;
    s_pos_cfg.i_max                = 5.0f;
    s_pos_cfg.out_max_deg          = 10.0f;   /* 倾角指令 ±10° */
    s_pos_cfg.invalid_reanchor_cnt = 50U;     /* 100Hz 下 50 = 0.5s */
    s_pos_cfg.dir_x                = 1.0f;    /* 台架验证后确定 ±1 */
    s_pos_cfg.dir_y                = 1.0f;    /* 台架验证后确定 ±1 */
}

/* =========================================================================
 * PID 对象
 * ========================================================================= */
PID_TypeDef pid_pitch;
PID_TypeDef pid_roll;

PID_TypeDef pid_rate_pitch;
PID_TypeDef pid_rate_roll;

/* 定高环：外环（高度差→速率目标） */
PID_TypeDef pid_alt;

/* 定高环：内环（速率差→油门修正） */
PID_TypeDef pid_alt_rate;

/* 定点环：横向速度环（速度差→横向倾角） */
PID_TypeDef pid_pos_x;

/* 定点环：纵向速度环（速度差→纵向倾角） */
PID_TypeDef pid_pos_y;

/* 目标高度(m)：由 Altitude_Stick_Input 摇杆积分，回中冻结 */
float Alt_Target_M = 0.0f;

/* 实测爬升速率(m/s)：距离差分+低通 */
float Alt_Rate_Mps = 0.0f;

/* 定高环油门偏差输出（DShot 单位）：= 内环 P+I 输出，限幅 ±s_alt_cfg.out_max。
 * Key==1 时叠加在油门摇杆 speed_temp 上；Key==3 时叠加在预设基准 ALT_DEV_OUT_MAX 上；
 * Key!=1/3（锁定/停机）时恒 0。 */
float Alt_Throttle_Out = 0.0f;

/* 偏航角速度环对象 */
PID_TypeDef pid_rate_yaw;

/* ── 定点环遥测（TelemetryTask 调试打印用） ── */
float   Pos_Vx_Mps    = 0.0f;  /* 实测横向速度(m/s)：flow_x×高度，正=向右 */
float   Pos_Vy_Mps    = 0.0f;  /* 实测纵向速度(m/s)：flow_y×高度，正=向机尾 */
float   Pos_SpX_Mps   = 0.0f;  /* 横向速度指令(m/s) */
float   Pos_SpY_Mps   = 0.0f;  /* 纵向速度指令(m/s) */
float   Pos_TiltX_Deg = 0.0f;  /* 横向倾角指令(deg) */
float   Pos_TiltY_Deg = 0.0f;  /* 纵向倾角指令(deg) */
uint8_t Pos_Active    = 0U;    /* 定点环是否生效（1=输出倾角指令，0=冻结/休眠） */

/* Pitch/Roll 串级对象 */
static PID_Cascade_t cascade_pitch;
static PID_Cascade_t cascade_roll;

/* 角速度低通：每轴一个独立实例，避免通道串扰 */
static LPF1_t gyro_pitch_lpf;
static LPF1_t gyro_roll_lpf;
static LPF1_t gyro_yaw_lpf;

/* ── 定高环内部状态 ─────────────────────────────────────────────── */
static LPF1_t    alt_rate_lpf;            /* 爬升速率低通 */
static uint8_t   s_alt_anchored = 0U;     /* 目标是否已锚定（解锁/回切/恢复后首帧有效距离） */
static uint16_t  s_alt_invalid_cnt = 0U;  /* 连续无新帧/无效帧计数（超阈值重锚定） */
static float     s_alt_last_dist_m = 0.0f; /* 上一帧距离(m)，微分用 */
static uint32_t  s_alt_last_time_ms = 0U;  /* 上一帧传感器时间戳(ms)：微分 dt + 新鲜度判断用 */

/* ── 定点环内部状态 ─────────────────────────────────────────────── */
static LPF1_t    pos_vel_x_lpf;           /* 横向速度低通 */
static LPF1_t    pos_vel_y_lpf;           /* 纵向速度低通 */
static uint8_t   s_pos_anchored = 0U;     /* 是否已锚定（解锁/恢复后首帧有效数据） */
static uint16_t  s_pos_invalid_cnt = 0U;  /* 连续无效/无新帧计数（超阈值复位冻结） */
static uint32_t  s_pos_last_time_ms = 0U; /* 上一帧传感器时间戳(ms)：新鲜度判断 */

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

	/* ---- 定高环 PID（100Hz，MTF02P ToF 距离；kp/ki 与别的环一样在 main.c Set_PID 调参） ---- */
	Alt_Config_Init();

	/* 外环：高度差 → 速率目标。纯 P，ki=0 故 Integral_max 传 0 */
	PID_Init(&pid_alt);
	PID_Init_WithLimit(&pid_alt, 0.0f, s_alt_cfg.rate_target_max);
	PID_SetDeadband(&pid_alt, s_alt_cfg.pos_deadband_m);

	/* 内环：速率差 → 油门偏差输出（P+I），Out_max=±out_max；
	 * error_sum 上限 = i_max：Key==1 无预设基准时 I 需能学满悬停油门
	 * （默认 ki=100 × i_max=8 = ±800 = out_max） */
	PID_Init(&pid_alt_rate);
	PID_Init_WithLimit(&pid_alt_rate, s_alt_cfg.i_max, s_alt_cfg.out_max);
	PID_SetDeadband(&pid_alt_rate, s_alt_cfg.rate_deadband_mps);
	PID_SetIntegralSeparation(&pid_alt_rate, s_alt_cfg.rate_sep_mps);

	/* ---- 爬升速率估计低通（距离差分 → 低通） ---- */
	LPF1_Init(&alt_rate_lpf, s_alt_cfg.rate_lpf_alpha, 0.0f);

	/* ---- 定点环 PID（100Hz，MTF02P 光流；kp/ki 与别的环一样在 main.c Set_PID 调参） ---- */
	Pos_Config_Init();

	/* 横向速度环：速度差 → 横向倾角（P+I），输出限幅 ±out_max_deg */
	PID_Init(&pid_pos_x);
	PID_Init_WithLimit(&pid_pos_x, s_pos_cfg.i_max, s_pos_cfg.out_max_deg);
	PID_SetDeadband(&pid_pos_x, s_pos_cfg.vel_deadband_mps);
	PID_SetIntegralSeparation(&pid_pos_x, s_pos_cfg.vel_sep_mps);

	/* 纵向速度环：速度差 → 纵向倾角（P+I） */
	PID_Init(&pid_pos_y);
	PID_Init_WithLimit(&pid_pos_y, s_pos_cfg.i_max, s_pos_cfg.out_max_deg);
	PID_SetDeadband(&pid_pos_y, s_pos_cfg.vel_deadband_mps);
	PID_SetIntegralSeparation(&pid_pos_y, s_pos_cfg.vel_sep_mps);

	/* ---- 机体速度估计低通（光流 × 高度 → 低通） ---- */
	LPF1_Init(&pos_vel_x_lpf, s_pos_cfg.vel_lpf_alpha, 0.0f);
	LPF1_Init(&pos_vel_y_lpf, s_pos_cfg.vel_lpf_alpha, 0.0f);
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

	/* ---- 定高环重置：目标待锚定（解锁后首帧有效距离即目标），输出归零 ---- */
	PID_Reset(&pid_alt);
	PID_Reset(&pid_alt_rate);
	LPF1_Init(&alt_rate_lpf, s_alt_cfg.rate_lpf_alpha, 0.0f);
	Alt_Target_M     = 0.0f;
	Alt_Rate_Mps     = 0.0f;
	Alt_Throttle_Out = 0.0f;
	s_alt_anchored     = 0U;
	s_alt_invalid_cnt  = 0U;
	s_alt_last_dist_m  = 0.0f;
	s_alt_last_time_ms = 0U;

	/* ---- 定点环重置：倾角指令归零回平，速度低通清零，待重新锚定 ---- */
	PID_Reset(&pid_pos_x);
	PID_Reset(&pid_pos_y);
	LPF1_Init(&pos_vel_x_lpf, s_pos_cfg.vel_lpf_alpha, 0.0f);
	LPF1_Init(&pos_vel_y_lpf, s_pos_cfg.vel_lpf_alpha, 0.0f);
	Target_Pitch       = 0.0f;
	Target_Roll        = 0.0f;
	Pos_Vx_Mps         = 0.0f;
	Pos_Vy_Mps         = 0.0f;
	Pos_SpX_Mps        = 0.0f;
	Pos_SpY_Mps        = 0.0f;
	Pos_TiltX_Deg      = 0.0f;
	Pos_TiltY_Deg      = 0.0f;
	Pos_Active         = 0U;
	s_pos_anchored     = 0U;
	s_pos_invalid_cnt  = 0U;
	s_pos_last_time_ms = 0U;
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

	/* 解锁边沿检测：Key 0/2 -> 1/3 时重置 PID 状态
	 * （空中 3 <-> 1 切换不重置，不扰姿态） */
	if (((Key == 1U) || (Key == 3U)) && ((last_key == 0U) || (last_key == 2U)))
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

/* =========================================================================
 * Alt_Control — 定高环（100Hz）
 *
 * 由 ControlTask 在解锁分支（Key==1/3）以 500Hz 调用，内部分频至 100Hz。
 *
 * 状态分工（Key 由遥控器 RxPacket[0] 下发，无独立飞行模式）：
 *   - Key==1（解锁）：本环生效，混控 base = speed_temp + Alt_Throttle_Out
 *     （油门摇杆打底，PID 出偏差修正）
 *   - Key==3（解锁+预设）：混控 base = ALT_DEV_OUT_MAX + Alt_Throttle_Out
 *     （预设基准油门出大力，PID 只出偏差修正）
 *   - Key==2（锁定）：本环不被调用，Motor_Test 缓降并清零输出
 *   Altitude_Stick_Input 推杆 → 速率积分进目标高度，回中冻结目标 = 保持当前高度。
 *
 * 数据一致性：distance/time_ms 为 uint32 单字、MTF02P_IsRangeValid() 读单字节，
 * 均原子可安全直读；跨帧混读最多差一帧，由跳变毛刺保护兜底。
 *
 * 失效策略：
 *   - 无新帧（传感器时间戳不变）/数据无效/毛刺：冻结输出（保持最后一拍油门，空中不掉油门）
 *   - 连续无新帧或无效超阈值：置未锚定，恢复后以当前高度重新锚定（无冲击恢复）
 *   - 锚定时内环积分清零（PID_Reset），从当前 base 起重新学真实悬停差额
 * ========================================================================= */
void Alt_Control(void)
{
	static uint8_t div = 0U;

	float    dist_m;
	float    diff_m;
	float    rc_rate;
	float    rate_target;
	float    dt_s;
	uint8_t  range_valid;
	uint32_t dist_mm;
	uint32_t dt_ms;
	uint32_t sensor_time_ms;
	int8_t   rh;

	/* ── 非解锁状态（Key==2 等）：定高环休眠，输出恒 0 ── */
	if ((Key != 1U) && (Key != 3U))
	{
		Alt_Throttle_Out = 0.0f;
		s_alt_anchored   = 0U;
		return;
	}

	div++;
	if (div < s_alt_cfg.loop_div)
	{
		return;   /* 100Hz 降采样 */
	}
	div = 0U;

	sensor_time_ms = mtf02p_data.time_ms;
	dist_mm        = mtf02p_data.distance;
	range_valid    = MTF02P_IsRangeValid();

	/* ── 新鲜度/有效性：无新帧（时间戳没变，传感器冻结）或数据无效 →
	 *    冻结输出（保持最后一拍油门）；连续超阈值 → 置未锚定，恢复时重新锚定 ── */
	if ((range_valid == 0U) || (dist_mm == 0U) || (sensor_time_ms == s_alt_last_time_ms))
	{
		s_alt_invalid_cnt++;
		if (s_alt_invalid_cnt > s_alt_cfg.invalid_reanchor_cnt)
		{
			s_alt_invalid_cnt = (uint16_t)(s_alt_cfg.invalid_reanchor_cnt + 1U); /* 封顶防回绕 */
			s_alt_anchored = 0U;
		}
		return;
	}
	s_alt_invalid_cnt = 0U;

	dist_m = (float)dist_mm / 1000.0f;

	/* ── 锚定：首帧有效数据 → 只做状态初始化（PID/低通清零 + 微分参考值播种）。
	 *    目标高度 t 不锚定距离 —— t 完全由 Altitude_Stick_Input 摇杆积分控制，
	 *    静止/拿动时 t 不会跟随距离跳变。 */
	if (s_alt_anchored == 0U)
	{
		PID_Reset(&pid_alt);
		PID_Reset(&pid_alt_rate);
		LPF1_Init(&alt_rate_lpf, s_alt_cfg.rate_lpf_alpha, 0.0f);
		s_alt_last_dist_m  = dist_m;
		s_alt_last_time_ms = sensor_time_ms;
		Alt_Rate_Mps       = 0.0f;
		s_alt_anchored     = 1U;
		return;
	}

	/* ── 爬升速率估计：距离差分 + 低通 ──
	 * dt 用传感器帧时间戳（无符号减法天然处理回绕），异常时回退固定步长；
	 * 距离未变时按 diff=0 送入低通（速率自然衰减到 0，不冻结在陈旧值）。
	 * 参考值每拍无条件更新：陈旧基线被消除（修复静止后首帧假速率尖峰），
	 * 且真实高度剧变不会被毛刺保护永久丢弃。 */
	dt_ms = sensor_time_ms - s_alt_last_time_ms;
	dt_s  = ((dt_ms != 0U) && (dt_ms <= 500U))
	        ? ((float)dt_ms / 1000.0f) : s_alt_cfg.loop_dt_s;
	diff_m = dist_m - s_alt_last_dist_m;
	s_alt_last_dist_m  = dist_m;
	s_alt_last_time_ms = sensor_time_ms;

	/* ── 毛刺保护：单帧跳变超阈值视为假数据，本拍不参与控制（输出保持）；
	 *    参考值已更新，下一帧即可恢复跟踪 ── */
	if ((diff_m > s_alt_cfg.dist_jump_m) || (diff_m < -s_alt_cfg.dist_jump_m))
	{
		return;
	}

	Alt_Rate_Mps = LPF1_Update(&alt_rate_lpf, diff_m / dt_s);

	/* ── Altitude_Stick_Input → 速率指令（回中死区） ── */
	rc_rate = 0.0f;
	rh      = Altitude_Stick_Input;
	if ((rh > s_alt_cfg.rc_deadband) || (rh < -s_alt_cfg.rc_deadband))
	{
		rc_rate = (float)rh * (s_alt_cfg.rc_max_rate / 100.0f);
	}

	/* ── 目标高度积分：t 完全由摇杆控制（回中冻结 = 保持当前高度），
	 *    与实测距离无关 —— 不锚定、不跟随距离，静止时 t 不会随 d 跳变 ── */
	Alt_Target_M += rc_rate * s_alt_cfg.loop_dt_s;
	if (Alt_Target_M < s_alt_cfg.target_min_m) { Alt_Target_M = s_alt_cfg.target_min_m; }
	if (Alt_Target_M > s_alt_cfg.target_max_m) { Alt_Target_M = s_alt_cfg.target_max_m; }

	/* ── 地面死区：dist ≤ ground_dist_m（ToF 量程下限，实测地面恒读 20mm）= 飞控在地面 ──
	 * 高度误差不参与（防落地后继续转桨/地面自爬升），内环清零输出归零；
	 * 摇杆前馈保留 → 推 Altitude_Stick_Input 直接经速率内环 P 出油门起飞，回中电机静止在 base。 */
	if (dist_m <= s_alt_cfg.ground_dist_m)
	{
		PID_Reset(&pid_alt_rate);
		Alt_Throttle_Out = 0.0f;
		rate_target      = rc_rate;
	}
	else
	{
		/* ── 外环：高度差 → 爬升速率目标（+ 摇杆前馈，爬升响应更快） ── */
		PID_SetTarget(&pid_alt, Alt_Target_M);
		rate_target = PID_CalcDt(&pid_alt, dist_m, s_alt_cfg.loop_dt_s) + rc_rate;
	}
	rate_target = Limit_Output(rate_target, s_alt_cfg.rate_target_max);

	/* ── 内环：速率差 → 油门偏差输出（P 瞬态出力，I 学悬停差额；
	 *    输出限幅 ±out_max、error_sum 上限 i_max 均由 PID 库保证） ── */
	PID_SetTarget(&pid_alt_rate, rate_target);
	Alt_Throttle_Out = PID_CalcDt(&pid_alt_rate, Alt_Rate_Mps, s_alt_cfg.loop_dt_s);
}

/* =========================================================================
 * Pos_Freeze — 定点环冻结：复位速度环 + 倾角指令归零（回平，安全）
 * ========================================================================= */
static void Pos_Freeze(void)
{
	PID_Reset(&pid_pos_x);
	PID_Reset(&pid_pos_y);
	Target_Pitch   = 0.0f;
	Target_Roll    = 0.0f;
	Pos_TiltX_Deg  = 0.0f;
	Pos_TiltY_Deg  = 0.0f;
	Pos_Active     = 0U;
	s_pos_anchored = 0U;
}

/* =========================================================================
 * Pos_Control — 定点环（100Hz）
 *
 * 由 ControlTask 在解锁分支（Key==1/3）以 500Hz 调用，内部分频至 100Hz。
 * 必须在 PID_Pitch_Roll_Combined 之前调用：本环写 Target_Pitch/Target_Roll，
 * 串级外环随后用最新倾角目标（未到 100Hz 节拍时保持上一拍倾角）。
 *
 * 状态分工（与定高环一致，无独立飞行模式）：
 *   Key==1/3（解锁）：本环生效；Key==2（锁定）：本环不被调用，倾角归零。
 *
 * 方向约定（用户实测，机头朝上方位）：
 *   flow_x 左→右为正 / flow_y 机头→机尾为正；
 *   Position_X_Stick_Input +100 = 向右（flow_x 正）
 *   Position_Y_Stick_Input +100 = 向前（机尾→机头，flow_y 负）
 *   混控矩阵（Motor.c）中 pitch 项驱动左/右差速（横向）、roll 项驱动前/后差速
 *   （纵向），故横向 → Target_Pitch、纵向 → Target_Roll；dir_x/dir_y 为方向修正。
 *
 * 数据一致性：distance/time_ms 为 uint32 单字、flow_x/flow_y 为 int16 单字、
 * MTF02P_IsRangeValid()/IsFlowValid() 读单字节，均原子可安全直读；
 * 跨帧混读最多差一帧，由速度跳变毛刺保护兜底。
 *
 * 失效策略（沿用定高环模式）：
 *   - 测距/光流无效、flow_quality 低于阈值、无新帧（时间戳不变）、高度出窗：
 *     本拍冻结（保持最后一拍倾角）；连续超阈值 → 复位并倾角归零（回平，安全）
 *   - 贴地（dist ≤ ground_dist_m）：立即归零（起飞/落地阶段电机姿态中立）
 *   - 恢复有效后首帧重新锚定：PID/低通清零，速度低通用当前值播种（无冲击恢复）
 * ========================================================================= */
void Pos_Control(void)
{
	static uint8_t div = 0U;

	float    dist_m;
	float    vx_raw;
	float    vy_raw;
	float    sp_x;
	float    sp_y;
	uint8_t  range_valid;
	uint8_t  flow_valid;
	uint32_t dist_mm;
	uint32_t sensor_time_ms;
	int8_t   sx;
	int8_t   sy;

	/* ── 非解锁状态（Key==2 等）：定点环休眠，倾角指令归零 ── */
	if ((Key != 1U) && (Key != 3U))
	{
		Target_Pitch   = 0.0f;
		Target_Roll    = 0.0f;
		Pos_TiltX_Deg  = 0.0f;
		Pos_TiltY_Deg  = 0.0f;
		Pos_Active     = 0U;
		s_pos_anchored = 0U;
		return;
	}

	div++;
	if (div < s_pos_cfg.loop_div)
	{
		return;   /* 100Hz 降采样（未到节拍：保持上一拍倾角目标） */
	}
	div = 0U;

	sensor_time_ms = mtf02p_data.time_ms;
	dist_mm        = mtf02p_data.distance;
	range_valid    = MTF02P_IsRangeValid();
	flow_valid     = MTF02P_IsFlowValid();
	dist_m         = (float)dist_mm / 1000.0f;

	/* ── 贴地/测距无效：立即归零（起飞前/落地后姿态中立，不等计数阈值） ── */
	if ((range_valid == 0U) || (dist_mm == 0U) || (dist_m <= s_alt_cfg.ground_dist_m))
	{
		Pos_Freeze();
		return;
	}

	/* ── 有效性：光流无效/质量低/无新帧（时间戳没变，传感器冻结）/高度出窗 →
	 *    本拍冻结（保持最后一拍倾角）；连续超阈值 → 复位并归零 ── */
	if ((flow_valid == 0U) ||
	    (mtf02p_data.flow_quality < s_pos_cfg.quality_threshold) ||
	    (sensor_time_ms == s_pos_last_time_ms) ||
	    (dist_m < s_pos_cfg.height_min_m) || (dist_m > s_pos_cfg.height_max_m))
	{
		s_pos_invalid_cnt++;
		if (s_pos_invalid_cnt > s_pos_cfg.invalid_reanchor_cnt)
		{
			s_pos_invalid_cnt = (uint16_t)(s_pos_cfg.invalid_reanchor_cnt + 1U); /* 封顶防回绕 */
			Pos_Freeze();
		}
		return;
	}
	s_pos_invalid_cnt = 0U;

	/* ── 机体速度估计：光流 cm/s@1m × 高度(m) / 100 → m/s。
	 *    正=向右（flow_x）/ 正=向机尾（flow_y），用户实测方向约定 ── */
	vx_raw = (float)mtf02p_data.flow_x * dist_m / 100.0f;
	vy_raw = (float)mtf02p_data.flow_y * dist_m / 100.0f;

	/* ── 锚定：恢复有效后的首帧 → PID/低通清零 + 速度低通播种当前值，
	 *    本拍不参与控制，下一拍正常进入（无冲击恢复） ── */
	if (s_pos_anchored == 0U)
	{
		PID_Reset(&pid_pos_x);
		PID_Reset(&pid_pos_y);
		LPF1_Init(&pos_vel_x_lpf, s_pos_cfg.vel_lpf_alpha, vx_raw);
		LPF1_Init(&pos_vel_y_lpf, s_pos_cfg.vel_lpf_alpha, vy_raw);
		Pos_Vx_Mps        = vx_raw;
		Pos_Vy_Mps        = vy_raw;
		s_pos_last_time_ms = sensor_time_ms;
		s_pos_anchored    = 1U;
		return;
	}

	/* ── 毛刺保护：单拍速度跳变超阈值视为假数据，本拍不参与控制（倾角保持）；
	 *    速度低通不吸收毛刺，下一帧即可恢复 ── */
	if ((vx_raw > (Pos_Vx_Mps + s_pos_cfg.vel_jump_mps)) ||
	    (vx_raw < (Pos_Vx_Mps - s_pos_cfg.vel_jump_mps)) ||
	    (vy_raw > (Pos_Vy_Mps + s_pos_cfg.vel_jump_mps)) ||
	    (vy_raw < (Pos_Vy_Mps - s_pos_cfg.vel_jump_mps)))
	{
		return;
	}

	Pos_Vx_Mps = LPF1_Update(&pos_vel_x_lpf, vx_raw);
	Pos_Vy_Mps = LPF1_Update(&pos_vel_y_lpf, vy_raw);
	s_pos_last_time_ms = sensor_time_ms;
	Pos_Active = 1U;

	/* ── Position_X/Y_Stick_Input → 速度指令（回中死区，回中=0=悬停定点）。
	 *   摇杆约定：X +100=向右（flow_x 正）；Y +100=向前（flow_y 负，取负号） ── */
	sp_x = 0.0f;
	sp_y = 0.0f;
	sx   = Position_X_Stick_Input;
	sy   = Position_Y_Stick_Input;
	if ((sx > s_pos_cfg.rc_deadband) || (sx < -s_pos_cfg.rc_deadband))
	{
		sp_x = (float)sx * (s_pos_cfg.rc_max_mps / 100.0f);
	}
	if ((sy > s_pos_cfg.rc_deadband) || (sy < -s_pos_cfg.rc_deadband))
	{
		sp_y = -(float)sy * (s_pos_cfg.rc_max_mps / 100.0f);
	}
	Pos_SpX_Mps = sp_x;
	Pos_SpY_Mps = sp_y;

	/* ── 速度环 PI：速度差 → 倾角指令（P 瞬态出力，I 学稳态风扰差额；
	 *    输出限幅 ±out_max_deg、error_sum 上限 i_max 均由 PID 库保证） ── */
	PID_SetTarget(&pid_pos_x, sp_x);
	Pos_TiltX_Deg = PID_CalcDt(&pid_pos_x, Pos_Vx_Mps, s_pos_cfg.loop_dt_s);
	PID_SetTarget(&pid_pos_y, sp_y);
	Pos_TiltY_Deg = PID_CalcDt(&pid_pos_y, Pos_Vy_Mps, s_pos_cfg.loop_dt_s);

	/* ── 倾角指令 → 姿态目标（复用 Pitch/Roll 串级外环）：
	 *   混控矩阵中 pitch 项驱动左/右差速（横向）、roll 项驱动前/后差速（纵向），
	 *   故横向 → Target_Pitch、纵向 → Target_Roll；dir_x/dir_y 台架验证后确定 ── */
	Target_Pitch = s_pos_cfg.dir_x * Pos_TiltX_Deg;
	Target_Roll  = s_pos_cfg.dir_y * Pos_TiltY_Deg;
}
