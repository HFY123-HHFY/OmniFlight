/*
 * STP-23L 乐动激光雷达传感器驱动 — 协议解析实现
 *
 * 协议帧结构（195 字节）：
 *   [0..3]   4 × 0xAA  帧头
 *   [4]      0x00      设备地址
 *   [5]      0x02      命令 ID (GET_DISTANCE)
 *   [6..7]   0x00,0x00  偏移地址
 *   [8..9]   uint16 LE 数据长度 (180)
 *   [10..189] 180 字节  12 点载荷 (每点 15 字节: distance/noise/peak/confidence/intg/reftof)
 *   [190..193] uint32 LE 时间戳
 *   [194]    uint8     CRC (字节 4~193 累加和 mod 256)
 *
 * 数据处理：12 点取平均，剔除 distance==0 的无效点。
 */

#include "STP23L.h"

/* ── 全局输出变量 ────────────────────────────────────── */
float    stp23l_distance    = 0.0f;
uint16_t stp23l_distance_mm = 0U;
uint32_t stp23l_frame_cnt   = 0U;

/* ── 内部 RX 环形缓冲区（两层缓冲：ISR → 缓冲区 → 主循环解析） ── */
#define STP23L_RX_BUF_SIZE 512U

static uint8_t  s_rx_buf[STP23L_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;  /* ISR 生产者写入位置 */
static volatile uint16_t s_rx_tail;  /* 主循环消费者读取位置 */

/* ── 内部状态机 ──────────────────────────────────────── */
typedef enum
{
	STP23L_ST_HEADER = 0,   /* 等待 4 字节帧头 0xAA             */
	STP23L_ST_ADDR,         /* 验证设备地址 0x00               */
	STP23L_ST_CMD,          /* 验证命令 ID 0x02               */
	STP23L_ST_OFFSET0,      /* 验证偏移地址 [0] 0x00           */
	STP23L_ST_OFFSET1,      /* 验证偏移地址 [1] 0x00           */
	STP23L_ST_LEN_L,        /* 数据长度低字节                   */
	STP23L_ST_LEN_H,        /* 数据长度高字节                   */
	STP23L_ST_DATA,         /* 180 字节载荷（12 点 × 15 字节） */
	STP23L_ST_TIMESTAMP0,   /* 时间戳 [0]                     */
	STP23L_ST_TIMESTAMP1,   /* 时间戳 [1]                     */
	STP23L_ST_TIMESTAMP2,   /* 时间戳 [2]                     */
	STP23L_ST_TIMESTAMP3,   /* 时间戳 [3]                     */
	STP23L_ST_CRC           /* CRC 校验                       */
} STP23L_State_t;

static STP23L_State_t s_state;                     /* 当前解析状态            */
static STP23L_Point_t s_points[STP23L_POINTS_PER_FRAME]; /* 12 点原始数据缓存 */
static uint8_t  s_header_cnt;                      /* 帧头 0xAA 已匹配计数     */
static uint8_t  s_crc;                             /* CRC 累加器              */
static uint16_t s_data_len;                        /* 帧头声明的数据长度       */
static uint16_t s_data_idx;                        /* DATA 态内已收字节数      */
static uint8_t  s_point_idx;                       /* 当前点索引 (0~11)        */

/* ── 公开 API ────────────────────────────────────────── */

/* 重置状态机到帧头搜索态 + 清空内部缓冲区。 */
void STP23L_Init(void)
{
	s_state       = STP23L_ST_HEADER;
	s_header_cnt  = 0U;
	s_crc         = 0U;
	s_data_len    = 0U;
	s_data_idx    = 0U;
	s_point_idx   = 0U;

	/* 清空内部 RX 缓冲区 */
	s_rx_head     = 0U;
	s_rx_tail     = 0U;
}

/*
 * ISR 调用：将 1 字节推入内部环形缓冲区。
 * 仅入队，不做协议解析。队满时丢弃新字节。
 * Cortex-M4 上 16-bit 读写原子，单生产者/单消费者无需关中断。
 */
void STP23L_RxPush(uint8_t byte)
{
	uint16_t next_head;

	next_head = (uint16_t)((s_rx_head + 1U) % STP23L_RX_BUF_SIZE);
	if (next_head != s_rx_tail)
	{
		s_rx_buf[s_rx_head] = byte;
		s_rx_head = next_head;
	}
	/* 队满：丢弃该字节，不阻塞 */
}

/* 前向声明：协议解析核心（定义在文件尾部）。 */
static uint8_t STP23L_FeedByte(uint8_t byte);

/*
 * 主循环调用：消费内部缓冲区中所有缓存的字节，
 * 逐字节完成协议解析。非阻塞 — 缓冲区空时立即返回。
 */
void STP23L_Task(void)
{
	uint16_t tail;

	/*
	 * 用局部变量快照 head，避免主循环每次循环读 volatile。
	 * 只在缓冲区非空时才逐字节处理。
	 */
	while (s_rx_tail != s_rx_head)
	{
		tail = s_rx_tail;
		STP23L_FeedByte(s_rx_buf[tail]);
		s_rx_tail = (uint16_t)((tail + 1U) % STP23L_RX_BUF_SIZE);
	}
}

/*
 * 解析 data 区域内一个点的 15 字节载荷。
 * 每点字节顺序（小端，低字节在前）：
 * [0:1] distance, [2:3] noise, [4:7] peak, [8] confidence,
 * [9:12] intg, [13:14] reftof
 */
static void STP23L_ParsePointByte(uint8_t byte_idx, uint8_t byte)
{
	STP23L_Point_t *p;

	p = &s_points[s_point_idx];

	switch (byte_idx)
	{
	case 0U:  /* distance 低字节（先无符号扩到 int16_t，避免 byte≥0x80 时符号扩展破坏值） */
		p->distance = (int16_t)((uint16_t)byte);
		break;
	case 1U:  /* distance 高字节，完成 distance */
		p->distance |= (int16_t)((uint16_t)byte << 8);
		break;
	case 2U:  /* noise 低字节 */
		p->noise = (uint16_t)byte;
		break;
	case 3U:  /* noise 高字节 */
		p->noise |= (uint16_t)((uint16_t)byte << 8);
		break;
	case 4U:  /* peak [0] */
		p->peak = (uint32_t)byte;
		break;
	case 5U:  /* peak [1] */
		p->peak |= (uint32_t)((uint32_t)byte << 8);
		break;
	case 6U:  /* peak [2] */
		p->peak |= (uint32_t)((uint32_t)byte << 16);
		break;
	case 7U:  /* peak [3] */
		p->peak |= (uint32_t)((uint32_t)byte << 24);
		break;
	case 8U:  /* confidence */
		p->confidence = byte;
		break;
	case 9U:  /* intg [0] */
		p->intg = (uint32_t)byte;
		break;
	case 10U: /* intg [1] */
		p->intg |= (uint32_t)((uint32_t)byte << 8);
		break;
	case 11U: /* intg [2] */
		p->intg |= (uint32_t)((uint32_t)byte << 16);
		break;
	case 12U: /* intg [3] */
		p->intg |= (uint32_t)((uint32_t)byte << 24);
		break;
	case 13U: /* reftof 低字节（先无符号扩到 int16_t，避免符号扩展） */
		p->reftof = (int16_t)((uint16_t)byte);
		break;
	case 14U: /* reftof 高字节，完成该点，推进点索引 */
		p->reftof |= (int16_t)((uint16_t)byte << 8);
		s_point_idx++;
		break;
	default:
		break;
	}
}

/*
 * 一帧完整接收后：12 点取平均，剔除 distance==0 的无效点。
 * 更新全局输出 stp23l_distance / stp23l_distance_mm。
 */
static void STP23L_ProcessFrame(void)
{
	uint8_t i;
	uint8_t valid_cnt;
	int32_t sum_distance;

	valid_cnt   = 0U;
	sum_distance = 0;

	for (i = 0U; i < STP23L_POINTS_PER_FRAME; i++)
	{
		if (s_points[i].distance != 0)
		{
			sum_distance += (int32_t)s_points[i].distance;
			valid_cnt++;
		}
	}

	if (valid_cnt > 0U)
	{
		stp23l_distance_mm = (uint16_t)((uint32_t)sum_distance / (uint32_t)valid_cnt);
		stp23l_distance    = (float)stp23l_distance_mm * 0.001f;
	}

	stp23l_frame_cnt++;
}

/* 回到帧头搜索态，header 计数从 0 开始。 */
static void STP23L_ResetToHeader(void)
{
	s_state      = STP23L_ST_HEADER;
	s_header_cnt = 0U;
}

/* CRC 校验失败或帧异常时丢弃当前帧，回到搜索态。 */
static void STP23L_AbortFrame(void)
{
	s_state      = STP23L_ST_HEADER;
	s_header_cnt = 0U;
	s_crc        = 0U;
}

/*
 * 喂入 1 字节原始串口数据。
 * 返回 1 表示一帧解析完成且 CRC 通过，全局距离变量已更新。
 */
static uint8_t STP23L_FeedByte(uint8_t byte)
{
	switch (s_state)
	{
	/* ── 帧头：连续 4 字节 0xAA ── */
	case STP23L_ST_HEADER:
		if (byte == STP23L_HEADER)
		{
			s_header_cnt++;
			if (s_header_cnt >= 4U)
			{
				s_state      = STP23L_ST_ADDR;
				s_header_cnt = 0U;
				s_crc        = 0U;
			}
		}
		else
		{
			s_header_cnt = 0U;
		}
		break;

	/* ── 设备地址：固定 0x00 ── */
	case STP23L_ST_ADDR:
		if (byte == STP23L_DEVICE_ADDRESS)
		{
			s_crc += byte;
			s_state = STP23L_ST_CMD;
		}
		else
		{
			STP23L_ResetToHeader();
		}
		break;

	/* ── 命令 ID：固定 0x02 (GET_DISTANCE) ── */
	case STP23L_ST_CMD:
		if (byte == STP23L_CMD_GET_DISTANCE)
		{
			s_crc += byte;
			s_state = STP23L_ST_OFFSET0;
		}
		else
		{
			STP23L_AbortFrame();
		}
		break;

	/* ── 偏移地址 [0]：固定 0x00 ── */
	case STP23L_ST_OFFSET0:
		if (byte == STP23L_CHUNK_OFFSET)
		{
			s_crc += byte;
			s_state = STP23L_ST_OFFSET1;
		}
		else
		{
			STP23L_AbortFrame();
		}
		break;

	/* ── 偏移地址 [1]：固定 0x00 ── */
	case STP23L_ST_OFFSET1:
		if (byte == STP23L_CHUNK_OFFSET)
		{
			s_crc += byte;
			s_state = STP23L_ST_LEN_L;
		}
		else
		{
			STP23L_AbortFrame();
		}
		break;

	/* ── 数据长度低字节 ── */
	case STP23L_ST_LEN_L:
		s_crc      += byte;
		s_data_len  = byte;
		s_state     = STP23L_ST_LEN_H;
		break;

	/* ── 数据长度高字节 ── */
	case STP23L_ST_LEN_H:
		s_crc      += byte;
		s_data_len |= (uint16_t)((uint16_t)byte << 8);
		s_data_idx  = 0U;
		s_point_idx = 0U;

		/*
		 * 不校验 data_len 具体值 — 不同固件版本可能返回 180 或 184。
		 * 帧结构固定：12 点 × 15 字节 + 4 时间戳 + 1 CRC。
		 * 与官方参考代码一致：data_len 仅记录不使用。
		 */
		s_state = STP23L_ST_DATA;
		break;

	/* ── 12 点 × 15 字节 = 180 字节载荷数据（固定，不随 data_len 变化） ── */
	case STP23L_ST_DATA:
		s_crc += byte;
		STP23L_ParsePointByte((uint8_t)(s_data_idx % STP23L_BYTES_PER_POINT), byte);
		s_data_idx++;

		if (s_data_idx >= STP23L_DATA_LENGTH)  /* 固定 180，不使用 s_data_len */
		{
			s_state = STP23L_ST_TIMESTAMP0;
		}
		break;

	/* ── 时间戳 [0..3]：累计 CRC 但不解析（当前未使用） ── */
	case STP23L_ST_TIMESTAMP0:
		s_crc  += byte;
		s_state = STP23L_ST_TIMESTAMP1;
		break;

	case STP23L_ST_TIMESTAMP1:
		s_crc  += byte;
		s_state = STP23L_ST_TIMESTAMP2;
		break;

	case STP23L_ST_TIMESTAMP2:
		s_crc  += byte;
		s_state = STP23L_ST_TIMESTAMP3;
		break;

	case STP23L_ST_TIMESTAMP3:
		s_crc  += byte;
		s_state = STP23L_ST_CRC;
		break;

	/* ── CRC 校验 ── */
	case STP23L_ST_CRC:
		if (byte == s_crc)
		{
			STP23L_ProcessFrame();
		}
		/* CRC 无论成功失败都回到 header 搜索态，准备下一帧 */
		STP23L_ResetToHeader();
		return (byte == s_crc) ? 1U : 0U;

	default:
		STP23L_ResetToHeader();
		break;
	}

	return 0U;
}
