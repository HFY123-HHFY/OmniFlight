#ifndef __MTF02P_H
#define __MTF02P_H

/*
 * MTF-02P 微空科技光流测距一体化传感器驱动 — Micolink 协议解析层
 *
 * 架构（两层缓冲，与 STP-23L 同构）：
 *   1) ISR 调用 MTF02P_RxPush(byte)  → 只入队到内部环形缓冲区（快）
 *   2) Mtf02pTask 调用 MTF02P_Task()  → 消费缓冲区 → 协议解析 → 刷新 mtf02p_data
 *
 * 不负责：串口初始化 / 中断服务（API / Control_Task 层已处理）。
 *
 * 用法：
 *   ISR 侧（Control_Task_USART_Callback）:
 *     if (id == MTF02P_USART_ID) MTF02P_RxPush((uint8_t)data);  // 只入队，不解析
 *
 *   任务侧（Mtf02pTask, 2ms 轮询）:
 *     MTF02P_Task();                       // 非阻塞消费 + 解析 + 刷新
 *     dist = mtf02p_data.distance;         // 距离 (mm)，0 表示不可用
 *     vx   = mtf02p_data.flow_x;           // 光流速度 (cm/s @1m)，实际速度 = vx × 高度(m)
 *
 * 数据一致性：
 *   mtf02p_data 由 Mtf02pTask 单任务写入，ControlTask 等更高优先级任务读取时
 *   可能被抢占撕裂。需要同时读取多个字段时，先快照 mtf02p_frame_cnt，读完再
 *   比对一次，不等则重读；单字段读取可直接使用。
 */

#include <stdint.h>
#include "usart.h"      /* USART4 / API_USART4 宏（API 层串口头文件） */

/* ── 串口选择：改这里统一切换 MTF-02P 的 TX/RX ────────────────── */
#define MTF02P_USART         USART4      /* 串口寄存器实例（My_Usart 层引用） */
#define MTF02P_USART_ID      API_USART4  /* 串口逻辑 ID：对应 API_USART_Id_t（应用层 RX 分发引用） */

/* ── Micolink 协议常量 ─────────────────────────────────────── */
#define MTF02P_HEAD            0xEFU  /* 帧头                                   */
#define MTF02P_DEV_ID          0x0FU  /* 设备 ID：MTF-02P 固定                  */
#define MTF02P_MSG_ID_RANGE    0x51U  /* 消息 ID：测距传感器数据帧              */
#define MTF02P_PAYLOAD_LEN     20U    /* 0x51 帧负载长度（协议固定 0x14）        */
#define MTF02P_MAX_PAYLOAD_LEN 64U    /* 解析器最大容忍负载长度，超长视为垃圾帧 */

/* ── 数据负载结构（Micolink 0x51 帧解析结果，小端序） ────────── */
typedef struct
{
	uint32_t time_ms;       /* 系统时间 (ms)                                       */
	uint32_t distance;      /* 距离值 (mm)：最小值为 2，0 表示数据不可用            */
	uint8_t  strength;      /* 信号强度：无单位，供调试                             */
	uint8_t  precision;     /* 距离精度：无单位，供调试                             */
	uint8_t  tof_status;    /* 测距状态：1 = 测距数据可用                           */
	uint8_t  reserved1;     /* 预留                                                */
	int16_t  flow_x;        /* 光流速度 X 轴 (cm/s @1m)：实际速度(cm/s) = flow_x × 高度(m) */
	int16_t  flow_y;        /* 光流速度 Y 轴 (cm/s @1m)：实际速度(cm/s) = flow_y × 高度(m) */
	uint8_t  flow_quality;  /* 光流质量：数值越大表示光流数据可信度越高             */
	uint8_t  flow_status;   /* 光流状态：1 = 光流数据可用                           */
	uint16_t reserved2;     /* 预留                                                */
} MTF02P_Data_t;

/* ── 对外全局输出 ──────────────────────────────────────────── */

/* 最新一帧解析结果（Mtf02pTask 刷新，见文件头数据一致性说明）。 */
extern MTF02P_Data_t mtf02p_data;

/* 成功接收并校验通过的完整帧计数（溢出自动回绕）。 */
extern uint32_t mtf02p_frame_cnt;

/* ── 公开 API ─────────────────────────────────────────────── */

/* 初始化协议解析状态机 + 内部缓冲区（上电后调用一次，调度器启动前）。 */
void MTF02P_Init(void);

/*
 * ISR 调用：将 1 字节推入内部环形缓冲区（仅入队，不解析）。
 * 快速、无阻塞，可在中断上下文中安全调用（FreeRTOS 不感知）。
 */
void MTF02P_RxPush(uint8_t byte);

/*
 * 任务上下文调用：消费内部缓冲区中所有缓存的字节，
 * 逐字节完成 Micolink 协议解析，帧校验通过时自动刷新 mtf02p_data。
 * 非阻塞 — 缓冲区空时立即返回。
 */
void MTF02P_Task(void);

/*
 * 测距数据是否可用：tof_status == 1 且 distance >= 2。
 * 定高用 distance 前先调用此函数做信号判断。
 */
uint8_t MTF02P_IsRangeValid(void);

/*
 * 光流数据是否可用：flow_status == 1。
 * 定点用 flow_x/flow_y 前先调用此函数做信号判断。
 */
uint8_t MTF02P_IsFlowValid(void);

#endif /* __MTF02P_H */
