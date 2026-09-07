#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

#include "PID/PID.h"
#include "Filter/Filter.h"
#include "MPU6050_Int.h"
#include "pwm.h"
#include "TB6612.h"
#include "Motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ±2000dps 量程下 MPU6050 陀螺灵敏度：16.4 LSB/(deg/s) */
#define GYRO_SENS_2000DPS (16.4f)

/* PID 输出加载到电机前的总限幅。 */
#define MOTOR_MIX_LIMIT (2047.0f)

/* 目标姿态/高度。 */
extern float Target_Pitch;
extern float Target_Roll;
extern float Target_Yaw;

/* 目标偏航角速度 (deg/s)。当前恒 0 用于消除自旋，后续由遥控器写入。 */
extern float Target_Yaw_Rate;

/* 外环 PID：角度。 */
extern PID_TypeDef pid_pitch;
extern PID_TypeDef pid_roll;

/* 内环 PID：角速度。 */
extern PID_TypeDef pid_rate_pitch;
extern PID_TypeDef pid_rate_roll;
extern PID_TypeDef pid_rate_yaw;

/* ── 定点环（MTF02P 光流，100Hz，Key==1 解锁 / Key==3 解锁+预设基准时生效） ──
 * 与定高环同构的位置+速度串级：光流速度(m/s) → 积分 → 位置估计；摇杆速率指令 →
 * 积分 → 位置目标；位置外环（纯 P：位置差→速度修正+摇杆前馈）→ 速度内环（P+I：
 * 速度差→倾角指令 ±out_max_deg）→ Target_Pitch/Target_Roll，复用 Pitch/Roll 串级外环。
 * 速度环不用差分（光流本身是速度），无"陈旧基线假尖峰"问题；
 * 毛刺/失效/低质量策略沿用定高环模式。 */

/* ── 定点环配置结构体 ──
 * PID 增益（kp/ki）不在这里：与别的环一致，在 main.c 用 Set_PID 调参；
 * 这里收定点环专属参数（节拍/高度窗/死区/限幅/抗扰）。
 * 默认值由 Control.c 的 Pos_Config_Init() 加载，需要时可在 main.c 覆盖个别字段。 */
typedef struct
{
    /* ── 节拍 ── */
    uint8_t  loop_div;             /* 500Hz 内分频 → 定点节拍（5 → 100Hz，与定高环一致） */
    float    loop_dt_s;            /* 定点环固定步长（PID dt） */

    /* ── 生效高度窗（光流在低空/高空误差大，与定高量程一致） ── */
    float    height_min_m;         /* 生效最低高度：低于此高度贴地噪声大 → 不参与控制 */
    float    height_max_m;         /* 生效最高高度：高于此高度标定失效 → 不参与控制 */

    /* ── Position_X/Y_Stick_Input 摇杆（不回中，回中=悬停定点） ── */
    float    rc_max_mps;           /* 满杆 ±100 → ±m/s 速率指令（积分进位置目标） */
    int8_t   rc_deadband;          /* 回中死区 ±N，防摇杆回中偏移 */

    /* ── 机体速度估计（光流 × 高度 → m/s） ── */
    float    vel_lpf_alpha;        /* 速度低通系数（光流噪声大） */
    float    vel_jump_mps;         /* 单拍速度跳变阈值：超出视为毛刺（本拍不参与控制） */
    uint8_t  quality_threshold;    /* flow_quality 低于此值视为纹理不可信 → 停用 */

    /* ── 速度环 PI（速度差 → 倾角指令） ── */
    float    vel_deadband_mps;     /* 速度死区：悬停微差当 0，防抖动 */
    float    vel_sep_mps;          /* 速度误差超此值暂停积分，防大机动积分污染 */
    float    i_max;                /* error_sum 上限 */
    float    out_max_deg;          /* 倾角指令限幅 ±deg（复用串级外环输入，默认 12°） */

    /* ── 位置外环（纯 P：位置差 → 速度修正；摇杆速率积分 → 位置目标） ── */
    float    pos_deadband_m;       /* 位置死区：悬停微差当 0，防抖动 */
    float    vel_cmd_max_mps;      /* 速度指令限幅 ±m/s（= 满杆速率 + 误差修正余量，同定高环） */
    float    target_max_m;         /* 位置目标/估计限幅 ±m（光流无绝对位置，锚定点即原点） */

    /* ── 抗扰 ── */
    uint16_t invalid_reanchor_cnt; /* 连续无效/低质量/无新帧计数阈值 → 复位并倾角归零
                                      （100Hz 下 50 = 0.5s，与定高环一致） */

    /* ── 倾角指令方向（台架测试后确定，+1 或 -1，同 MOTOR_YAW_DIR 模式） ── */
    float    dir_x;                /* 横向倾角指令方向：Target_Pitch = dir_x × 横向输出 */
    float    dir_y;                /* 纵向倾角指令方向：Target_Roll  = dir_y × 纵向输出 */
} Pos_Cfg_t;

/* 定点环运行配置实例（默认值由 PID_Contorl_Init 内的 Pos_Config_Init 加载） */
extern Pos_Cfg_t s_pos_cfg;

/* 定点环位置外环 PID：位置差(m) → 速度修正(m/s)，纯 P；
 * 输出 + 摇杆前馈 → 速度内环指令（限幅 vel_cmd_max_mps） */
extern PID_TypeDef pid_pos_x;  /* 横向（左/右）位置环 */
extern PID_TypeDef pid_pos_y;  /* 纵向（前/后）位置环 */

/* 定点环速度内环 PID：速度差(m/s) → 倾角指令(deg)，P+I，
 * 输出经 dir_x/dir_y 方向修正后写入 Target_Pitch/Target_Roll */
extern PID_TypeDef pid_vel_x;  /* 横向（左/右）速度环 */
extern PID_TypeDef pid_vel_y;  /* 纵向（前/后）速度环 */

/* ── 定点环遥测（TelemetryTask 调试打印用） ── */
extern float   Pos_Vx_Mps;   /* 实测横向速度(m/s)：flow_x×高度，正=向右 */
extern float   Pos_Vy_Mps;   /* 实测纵向速度(m/s)：flow_y×高度，正=向机尾 */
extern float   Pos_SpX_Mps;  /* 横向速度指令(m/s)：= 位置外环输出 + 摇杆前馈（限幅后） */
extern float   Pos_SpY_Mps;  /* 纵向速度指令(m/s) */
extern float   Pos_EstX_M;   /* 横向位置估计(m)：实测速度积分，正=向右，锚定点为原点 */
extern float   Pos_EstY_M;   /* 纵向位置估计(m)：正=向机尾 */
extern float   Pos_TarX_M;   /* 横向位置目标(m)：摇杆速率积分，回中冻结 */
extern float   Pos_TarY_M;   /* 纵向位置目标(m) */
extern float   Pos_TiltX_Deg;/* 横向倾角指令(deg) */
extern float   Pos_TiltY_Deg;/* 纵向倾角指令(deg) */
extern uint8_t Pos_Active;   /* 定点环是否生效（1=输出倾角指令，0=冻结/休眠） */

/* ── 定高环（MTF02P ToF 距离，100Hz，Key==1 解锁 / Key==3 解锁+预设基准时生效） ── */

/* 定高环唯一宏：预设基准油门(DShot 单位)，约悬停油门（实测 400 偏小飞不起来，已调大）。
 * Key==3 = Key==1（解锁，Altitude_Stick_Input 控高度）+ 把本基准喂进混控 base，PID 只出偏差修正（不累）；
 * Key==1 有油门摇杆 speed_temp 打底（base = speed_temp + 偏差输出），不经本基准。
 * 改这一个宏 = 改给高度环的基准油门。 */
#define ALT_DEV_OUT_MAX (600.0f)

/* ── 定高环配置结构体 ──
 * PID 增益（kp/ki）不在这里：与 Pitch/Roll/偏航环一致，在 main.c 用 Set_PID 调参；
 * 这里收定高环专属参数（节拍/死区/限幅/抗扰）。
 * 默认值由 Control.c 的 Alt_Config_Init() 加载，需要时可在 main.c 覆盖个别字段。 */
typedef struct
{
    /* ── 节拍 ── */
    uint8_t  loop_div;             /* 500Hz 内分频 → 定高节拍（5 → 100Hz） */
    float    loop_dt_s;            /* 定高环固定步长 10ms（PID dt） */

    /* ── Altitude_Stick_Input 摇杆 ── */
    float    rc_max_rate;          /* 满杆 ±100 → ±m/s 速率指令 */
    int8_t   rc_deadband;          /* 回中死区 ±N，防摇杆回中偏移 */

    /* ── 高度外环（纯 P：高度差 → 速率目标） ── */
    float    pos_deadband_m;       /* 高度死区：悬停微差当 0，防抖动 */
    float    rate_target_max;      /* 速率目标限幅 ±m/s（= 满杆速率 + 误差修正余量） */

    /* ── 速率内环（P+I：速率差 → 油门偏差） ── */
    float    i_max;                /* error_sum 上限：I_out ≤ ki×i_max（默认 ki=150×8=±1200=out_max，
                                      I 需能学满悬停差额） */
    float    out_max;              /* 油门偏差输出限幅 ±（DShot 单位） */
    float    rate_deadband_mps;    /* 速率死区：微分噪声当 0，I 保持悬停修正 */
    float    rate_sep_mps;         /* 速率误差超此值暂停积分，防大机动积分污染 */

    /* ── 速率估计 ── */
    float    rate_lpf_alpha;       /* 距离微分低通系数（ToF 单帧微分噪声大） */

    /* ── 抗扰 ── */
    float    dist_jump_m;          /* 单帧距离跳变阈值：超出视为毛刺（本拍不参与控制） */
    uint16_t invalid_reanchor_cnt; /* 连续无新帧/无效帧计数阈值 → 置未锚定（100Hz 下 50=0.5s） */
    float    ground_dist_m;        /* 地面死区：dist ≤ 此值视为在地面（ToF 量程下限，实测 20mm 恒读） */
    float    target_min_m;         /* 目标高度下限 = 地面值：解锁 t 从地面起，向下推杆最低降到地面 */
    float    target_max_m;         /* 目标高度上限（ToF 可靠量程内，可调） */
} Alt_Cfg_t;

/* 定高环运行配置实例（默认值由 PID_Contorl_Init 内的 Alt_Config_Init 加载） */
extern Alt_Cfg_t s_alt_cfg;

/* 外环：高度差(m) → 爬升速率目标(m/s)，纯 P */
extern PID_TypeDef pid_alt;

/* 内环：爬升速率差(m/s) → 油门偏差输出(DShot 单位)，P+I，
 * Key==1 叠加在油门摇杆 speed_temp 上 / Key==3 叠加在预设基准 ALT_DEV_OUT_MAX 上 */
extern PID_TypeDef pid_alt_rate;

/* 目标高度(m)：由 R_H 摇杆指令积分而来，摇杆回中即冻结（保持当前高度） */
extern float Alt_Target_M;

/* 实测爬升速率(m/s)：距离差分+低通，调试遥测用 */
extern float Alt_Rate_Mps;

/* 定高环油门偏差输出(DShot 单位)：= 内环 P+I 输出，限幅 ±s_alt_cfg.out_max。
 * Key==1：叠加在油门摇杆 speed_temp 上；Key==3：叠加在预设基准 ALT_DEV_OUT_MAX 上；
 * Key==2（锁定/停机）时恒 0。 */
extern float Alt_Throttle_Out;

/* 定高环是否生效（1=正在控制高度，0=休眠/未锚定/贴地）。
 * 与 Pos_Active 成对：NRF24L01 回传遥控器，一眼可见定高/定点是否在工作。 */
extern uint8_t Alt_Active;

/*
 * 控制初始化：
 * 1) 初始化外环/内环 PID（含限幅）
 * 2) 初始化串级控制对象
 * 3) 初始化陀螺低通滤波器
 */
void PID_Contorl_Init(void);

/*
 * 陀螺零偏校准（上电后调用一次，飞行器必须静止）。
 * samples: 采样点数（建议 1000，约 5s）。
 * 同时校准 X/Y/Z 三轴，Z 轴零偏供偏航角速度环使用。
 * 返回 1 完成（采样不再依赖 EXTI 标志，无超时概念）。
 */
uint8_t GyroBias_Calibrate(uint16_t samples);

/* 查询校准是否完成。 */
uint8_t GyroBias_IsReady(void);

/* 手动设置陀螺零偏（仅 X/Y 轴，单位：原始 LSB）。 */
void Set_Gyro_Bias(float bias_x, float bias_y);

/* 获取 Z 轴陀螺零偏（原始 LSB），调试用。 */
float Get_Gyro_Bias_Z(void);

/* 解锁时重置 PID 内部状态 + 低通滤波器，防止地面噪声污染导致解锁瞬态。 */
void Control_Arm_Reset(float current_gyro_pitch_dps, float current_gyro_roll_dps);

/*
 * Pitch/Roll 串级 PID 控制。
 * 校准已独立完成，此函数不再包含校准逻辑。
 */
void PID_Pitch_Roll_Combined(float actual_pitch, float actual_roll);

/*
 * 偏航角速度环 PID 控制（单环，500Hz）。
 * gyro_z_dps: Z 轴角速度（deg/s），已由调用方转换为 deg/s，内部再做零偏扣除。
 * 输出写入 pid_rate_yaw.output，由混控层加载 yaw 项。
 */
void PID_Yaw_Rate_Control(float gyro_z_dps);

/*
 * 定高环控制（100Hz，内部对 500Hz 调用做分频）。
 *
 * 仅在解锁状态生效：Key==1（解锁）/ Key==3（解锁+预设基准油门）：
 *   R_H 摇杆 → 速率指令积分 → Alt_Target_M（回中冻结目标）
 *   高度外环 pid_alt：高度差 → 爬升速率目标（+ 摇杆前馈）
 *   速率内环 pid_alt_rate：速率差 → 油门偏差输出（P+I，±out_max 限幅）
 *   Alt_Throttle_Out = 内环偏差 → 混控 base（Motor.c）：
 *     Key==1 = speed_temp + Alt_Throttle_Out（油门摇杆打底，R_H 定高环出偏差）
 *     Key==3 = ALT_DEV_OUT_MAX + Alt_Throttle_Out（预设基准，PID 只出偏差）
 *
 * Key==2（锁定）：本环不被调用，Motor_Test 缓降并清零输出。
 * 参数：唯一宏 ALT_DEV_OUT_MAX（预设基准油门）+ Alt_Cfg_t 结构体（节拍/死区/限幅/抗扰）；
 * PID 增益与 Pitch/Roll/偏航环一致，在 main.c 用 Set_PID 调参。
 * 锚定时内环积分清零；数据失效/毛刺/无新帧时冻结输出（保持最后一拍油门）。
 */
void Alt_Control(void);

/*
 * 定点环控制（100Hz，内部对 500Hz 调用做分频）。
 *
 * 仅在解锁状态生效：Key==1（解锁）/ Key==3（解锁+预设基准油门）：
 *   Position_X/Y_Stick_Input 摇杆 → 速率指令积分 → 位置目标（回中冻结=悬停当前位置）
 *   MTF02P 光流 × 高度 → 实测机体速度（低通+毛刺保护）→ 积分 → 位置估计
 *   位置外环 pid_pos_x/y（纯 P）：位置差 → 速度修正（+ 摇杆前馈，限幅）
 *   速度内环 pid_vel_x/y（P+I）：速度差 → 倾角指令（±out_max_deg 限幅）
 *   倾角指令经 dir_x/dir_y 方向修正写入 Target_Pitch（横向）/Target_Roll（纵向），
 *   复用 Pitch/Roll 串级外环（PID_Pitch_Roll_Combined）
 *
 * 方向约定（用户实测，机头朝上方位）：flow_x 左→右为正 / flow_y 机头→机尾为正；
 *   Position_X_Stick_Input +100 = 向右（flow_x 正）；Position_Y_Stick_Input +100 =
 *   向前 = 机尾→机头（flow_y 负）。
 *
 * 失效策略（沿用定高环模式）：
 *   测距/光流无效、flow_quality 低于阈值、无新帧（时间戳不变）、高度出窗：
 *   本拍冻结（保持最后一拍倾角）；连续超阈值 → 复位并倾角归零（回平，安全）。
 *   贴地（dist ≤ ground_dist_m）立即归零；恢复有效后首帧重新锚定（PID/低通清零）。
 *   锚定/冻结即重建位置坐标系（光流无绝对位置，位置目标与估计一并清零）。
 * 参数：Pos_Cfg_t 结构体（节拍/高度窗/死区/限幅/抗扰/方向）；
 * PID 增益与别的环一致，在 main.c 用 Set_PID 调参。
 */
void Pos_Control(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
