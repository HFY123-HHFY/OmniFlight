/*
 * MTF-02P 微空科技光流测距一体化传感器驱动 — Micolink 协议解析实现
 *
 * Micolink 帧结构（一帧共 7 + len 字节）：
 *   [0]      0xEF      帧头
 *   [1]      0x0F      设备 ID（MTF-02P 固定）
 *   [2]      0x00      系统 ID
 *   [3]      0x51      消息 ID（测距传感器）
 *   [4]      0~0xFF    包序列
 *   [5]      len       负载长度（0x51 帧固定 0x14 = 20）
 *   [6..]    payload   数据负载（见下方布局）
 *   [最后]   checksum  前面所有字节累加和（uint8 自然回绕）
 *
 * 数据负载（20 字节，小端序）：
 *   [0:4]   uint32 time_ms        系统时间
 *   [4:8]   uint32 distance       距离 (mm)，0 = 不可用
 *   [8]     uint8  strength       信号强度
 *   [9]     uint8  precision      距离精度
 *   [10]    uint8  tof_status     测距状态，1 = 可用
 *   [11]    uint8  reserved1      预留
 *   [12:14] int16  flow_vel_x     光流速度 X (cm/s @1m)
 *   [14:16] int16  flow_vel_y     光流速度 Y (cm/s @1m)
 *   [16]    uint8  flow_quality   光流质量，越大越可信
 *   [17]    uint8  flow_status    光流状态，1 = 可用
 *   [18:20] uint16 reserved2      预留
 */

#include "MTF02P.h"

/* ── 全局输出变量 ────────────────────────────────────────── */
MTF02P_Data_t mtf02p_data    = {0U};
uint32_t      mtf02p_frame_cnt = 0U;

/* ── 内部 RX 环形缓冲区（两层缓冲：ISR → 缓冲区 → 任务解析） ── */
#define MTF02P_RX_BUF_SIZE 256U   /* 115200 波特率、27B/帧、帧率 ≤100Hz，256B 余量充足 */

static uint8_t  s_rx_buf[MTF02P_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;  /* ISR 生产者写入位置 */
static volatile uint16_t s_rx_tail;  /* 任务消费者读取位置 */

/* ── 内部解析状态机 ──────────────────────────────────────── */
typedef enum
{
	MTF02P_ST_HEAD = 0,  /* 等待帧头 0xEF                    */
	MTF02P_ST_DEV_ID,    /* 验证设备 ID 0x0F                */
	MTF02P_ST_SYS_ID,    /* 系统 ID（不校验，仅累计校验和）  */
	MTF02P_ST_MSG_ID,    /* 消息 ID                          */
	MTF02P_ST_SEQ,       /* 包序列（仅累计校验和）           */
	MTF02P_ST_LEN,       /* 负载长度                         */
	MTF02P_ST_PAYLOAD,   /* 数据负载                         */
	MTF02P_ST_CHECKSUM   /* 帧校验                           */
} MTF02P_State_t;

static MTF02P_State_t s_state;                        /* 当前解析状态        */
static uint8_t  s_payload[MTF02P_MAX_PAYLOAD_LEN];    /* 本帧负载字节缓存    */
static uint8_t  s_checksum;                           /* 校验和累加器        */
static uint8_t  s_msg_id;                             /* 本帧消息 ID         */
static uint8_t  s_len;                                /* 本帧声明的负载长度  */
static uint8_t  s_payload_idx;                        /* 已收负载字节数      */

/* ── 公开 API ────────────────────────────────────────────── */

/* 重置状态机到帧头搜索态 + 清空内部缓冲区。 */
void MTF02P_Init(void)
{
	s_state       = MTF02P_ST_HEAD;
	s_checksum    = 0U;
	s_msg_id      = 0U;
	s_len         = 0U;
	s_payload_idx = 0U;

	/* 清空内部 RX 缓冲区 */
	s_rx_head     = 0U;
	s_rx_tail     = 0U;
}

/*
 * ISR 调用：将 1 字节推入内部环形缓冲区。
 * 仅入队，不做协议解析。队满时丢弃新字节。
 * Cortex-M4 上 16-bit 读写原子，单生产者/单消费者无需关中断。
 */
void MTF02P_RxPush(uint8_t byte)
{
	uint16_t next_head;

	next_head = (uint16_t)((s_rx_head + 1U) % MTF02P_RX_BUF_SIZE);
	if (next_head != s_rx_tail)
	{
		s_rx_buf[s_rx_head] = byte;
		s_rx_head = next_head;
	}
	/* 队满：丢弃该字节，不阻塞 */
}

/* 前向声明：协议解析核心（定义在文件尾部）。 */
static uint8_t MTF02P_FeedByte(uint8_t byte);

/*
 * 任务上下文调用：消费内部缓冲区中所有缓存的字节，
 * 逐字节完成协议解析。非阻塞 — 缓冲区空时立即返回。
 */
void MTF02P_Task(void)
{
	uint16_t tail;

	/*
	 * 用局部变量快照 head，避免每次循环读 volatile。
	 * 只在缓冲区非空时才逐字节处理。
	 */
	while (s_rx_tail != s_rx_head)
	{
		tail = s_rx_tail;
		MTF02P_FeedByte(s_rx_buf[tail]);
		s_rx_tail = (uint16_t)((tail + 1U) % MTF02P_RX_BUF_SIZE);
	}
}

/* ── 信号判断辅助 ────────────────────────────────────────── */

/* 测距数据可用：tof_status == 1 且 distance >= 2（0 为协议约定的不可用值）。 */
uint8_t MTF02P_IsRangeValid(void)
{
	if ((mtf02p_data.tof_status == 1U) && (mtf02p_data.distance >= 2U))
	{
		return 1U;
	}
	return 0U;
}

/* 光流数据可用：flow_status == 1。 */
uint8_t MTF02P_IsFlowValid(void)
{
	if (mtf02p_data.flow_status == 1U)
	{
		return 1U;
	}
	return 0U;
}

/* ── 内部解析函数 ────────────────────────────────────────── */

/* 小端序解析：4 字节无符号整数。 */
static uint32_t MTF02P_ReadU32(const uint8_t *p)
{
	return (uint32_t)p[0]
	     | ((uint32_t)p[1] << 8)
	     | ((uint32_t)p[2] << 16)
	     | ((uint32_t)p[3] << 24);
}

/* 小端序解析：2 字节有符号整数（光流速度可为负）。 */
static int16_t MTF02P_ReadI16(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * 一帧完整接收且校验通过后：解析 0x51 负载，刷新 mtf02p_data。
 * 非 0x51 帧仅计数不解析；负载长度小于 20 字节时视为异常帧，不更新数据。
 */
static void MTF02P_ProcessFrame(void)
{
	if ((s_msg_id == MTF02P_MSG_ID_RANGE) && (s_len >= MTF02P_PAYLOAD_LEN))
	{
		mtf02p_data.time_ms      = MTF02P_ReadU32(&s_payload[0]);
		mtf02p_data.distance     = MTF02P_ReadU32(&s_payload[4]);
		mtf02p_data.strength     = s_payload[8];
		mtf02p_data.precision    = s_payload[9];
		mtf02p_data.tof_status   = s_payload[10];
		mtf02p_data.reserved1    = s_payload[11];
		mtf02p_data.flow_x       = MTF02P_ReadI16(&s_payload[12]);
		mtf02p_data.flow_y       = MTF02P_ReadI16(&s_payload[14]);
		mtf02p_data.flow_quality = s_payload[16];
		mtf02p_data.flow_status  = s_payload[17];
		mtf02p_data.reserved2    = (uint16_t)s_payload[18] | ((uint16_t)s_payload[19] << 8);
	}

	mtf02p_frame_cnt++;
}

/* 回到帧头搜索态，校验和与负载索引清零。 */
static void MTF02P_ResetToHeader(void)
{
	s_state       = MTF02P_ST_HEAD;
	s_checksum    = 0U;
	s_len         = 0U;
	s_payload_idx = 0U;
}

/* 帧校验失败或帧异常时丢弃当前帧，回到搜索态。 */
static void MTF02P_AbortFrame(void)
{
	MTF02P_ResetToHeader();
}

/*
 * 喂入 1 字节原始串口数据。
 * 返回 1 表示一帧解析完成且校验通过，mtf02p_data 可能已更新。
 */
static uint8_t MTF02P_FeedByte(uint8_t byte)
{
	switch (s_state)
	{
	/* ── 帧头：0xEF ── */
	case MTF02P_ST_HEAD:
		if (byte == MTF02P_HEAD)
		{
			s_checksum = byte;   /* 校验和从帧头开始累计 */
			s_state    = MTF02P_ST_DEV_ID;
		}
		break;

	/* ── 设备 ID：MTF-02P 固定 0x0F ── */
	case MTF02P_ST_DEV_ID:
		if (byte == MTF02P_DEV_ID)
		{
			s_checksum += byte;
			s_state     = MTF02P_ST_SYS_ID;
		}
		else
		{
			MTF02P_AbortFrame();
		}
		break;

	/* ── 系统 ID：协议固定 0x00，只累计不校验 ── */
	case MTF02P_ST_SYS_ID:
		s_checksum += byte;
		s_state     = MTF02P_ST_MSG_ID;
		break;

	/* ── 消息 ID ── */
	case MTF02P_ST_MSG_ID:
		s_checksum += byte;
		s_msg_id    = byte;
		s_state     = MTF02P_ST_SEQ;
		break;

	/* ── 包序列：只累计，不存储 ── */
	case MTF02P_ST_SEQ:
		s_checksum += byte;
		s_state     = MTF02P_ST_LEN;
		break;

	/* ── 负载长度 ── */
	case MTF02P_ST_LEN:
		s_checksum    += byte;
		s_len          = byte;
		s_payload_idx  = 0U;

		if (s_len == 0U)
		{
			s_state = MTF02P_ST_CHECKSUM;   /* 空负载帧，直接等校验 */
		}
		else if (s_len > MTF02P_MAX_PAYLOAD_LEN)
		{
			MTF02P_AbortFrame();            /* 超长帧视为垃圾数据 */
		}
		else
		{
			s_state = MTF02P_ST_PAYLOAD;
		}
		break;

	/* ── 数据负载：收满 len 字节后进入校验态 ── */
	case MTF02P_ST_PAYLOAD:
		s_checksum += byte;
		s_payload[s_payload_idx++] = byte;
		if (s_payload_idx >= s_len)
		{
			s_state = MTF02P_ST_CHECKSUM;
		}
		break;

	/* ── 帧校验：与累加和比对 ── */
	case MTF02P_ST_CHECKSUM:
		if (byte == s_checksum)
		{
			MTF02P_ProcessFrame();
			MTF02P_ResetToHeader();
			return 1U;
		}

		/*
		 * 校验失败。若该字节恰好是下一帧的帧头 0xEF，说明误同步丢失了
		 * 一帧边界，直接把它当作新帧开头继续解析，少丢一帧。
		 */
		if (byte == MTF02P_HEAD)
		{
			s_checksum = byte;
			s_state    = MTF02P_ST_DEV_ID;
		}
		else
		{
			MTF02P_ResetToHeader();
		}
		break;

	default:
		MTF02P_ResetToHeader();
		break;
	}

	return 0U;
}
