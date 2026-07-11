#ifndef __STP23L_H
#define __STP23L_H

/*
 * STP-23L 乐动激光雷达传感器驱动 — 协议解析层
 *
 * 架构（两层缓冲）：
 *   1) ISR 调用 STP23L_RxPush(byte)  → 只入队到内部环形缓冲区（快）
 *   2) 主循环调用 STP23L_Task()      → 消费缓冲区 → 协议解析 → 刷新距离
 *
 * 不负责：串口初始化 / 中断服务（API / Control_Task 层已处理）。
 *
 * 用法：
 *   ISR 侧（Control_Task_USART_Callback）:
 *     STP23L_RxPush((uint8_t)data);  // 只入队，不解析
 *
 *   主循环侧（main.c）:
 *     STP23L_Task();                 // 非阻塞消费 + 解析 + 刷新
 *     distance = stp23l_distance;    // 直接读取 (m)
 */

#include <stdint.h>

/* ── 协议常量 ─────────────────────────────────────────── */
#define STP23L_HEADER           0xAAU  /* 帧头           */
#define STP23L_DEVICE_ADDRESS   0x00U  /* 设备地址        */
#define STP23L_CMD_GET_DISTANCE 0x02U  /* 获取距离数据命令 */
#define STP23L_CHUNK_OFFSET     0x00U  /* 偏移地址        */
#define STP23L_POINTS_PER_FRAME 12U    /* 每帧测量点数     */
#define STP23L_BYTES_PER_POINT  15U    /* 每点数据字节数   */
#define STP23L_DATA_LENGTH      (STP23L_POINTS_PER_FRAME * STP23L_BYTES_PER_POINT) /* 180 */

/* ── 单点数据结构（协议中 15 字节载荷的小端解析结果） ── */
typedef struct
{
	int16_t  distance;    /* 距离 (mm)，核心数据                  */
	uint16_t noise;       /* 环境光噪声，越大干扰越强             */
	uint32_t peak;        /* 激光回波强度                         */
	uint8_t  confidence;  /* 置信度，融合环境光和回波强度的指标     */
	uint32_t intg;        /* 积分次数                             */
	int16_t  reftof;      /* 温度参考值（芯片内部温度补偿值）       */
} STP23L_Point_t;

/* ── 对外全局输出 ─────────────────────────────────────── */

/* 融合距离 (m)，供飞控高度环直接使用。每帧解析成功后更新。 */
extern float stp23l_distance;

/* 融合距离 (mm)，供串口打印/调试。 */
extern uint16_t stp23l_distance_mm;

/* 成功接收并 CRC 校验通过的完整帧计数（溢出自动回绕）。 */
extern uint32_t stp23l_frame_cnt;

/* ── 公开 API ────────────────────────────────────────── */

/* 初始化协议解析状态机 + 内部缓冲区（上电后调用一次）。 */
void STP23L_Init(void);

/*
 * ISR 调用：将 1 字节推入内部环形缓冲区（仅入队，不解析）。
 * 快速、无阻塞，可在中断上下文中安全调用。
 */
void STP23L_RxPush(uint8_t byte);

/*
 * 主循环调用：消费内部缓冲区中所有缓存的字节，
 * 逐字节完成协议解析，帧完整时自动刷新 stp23l_distance。
 * 非阻塞 — 缓冲区空时立即返回。
 */
void STP23L_Task(void);

#endif /* __STP23L_H */
