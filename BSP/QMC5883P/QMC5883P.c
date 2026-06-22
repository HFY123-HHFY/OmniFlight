#include "QMC5883P.h"

#include "API_I2C.h"
#include "LED.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include <math.h>

/* 最近一次角度结果（单位：度，0~360）。 */
float Angle_XY = 0.0f;

/*  选择I2C1 设置QMC5883P I2C速率为400kHZ */
static void QMC_SelectI2CSpeed(void)
{
	API_I2C_SelectBus(QMC5883P_I2C_BUS);
	API_I2C_SetSpeed(QMC5883P_I2C_SPEED);
}

/* 向指定寄存器写 1 字节。 */
static void QMC_WriteReg(uint8_t regAddress, uint8_t data)
{
	QMC_SelectI2CSpeed();
	API_I2C_Start();
	API_I2C_SendByte(QMC5883P_I2C_ADDR_W);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(regAddress);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(data);
	API_I2C_Wait_Ack();
	API_I2C_Stop();
}

/* 从指定寄存器读取 1 字节。 */
static uint8_t QMC_ReadReg(uint8_t regAddress)
{
	uint8_t data;

	QMC_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(QMC5883P_I2C_ADDR_W);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(regAddress);
	API_I2C_Wait_Ack();

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(QMC5883P_I2C_ADDR_W | 0x01U));
	API_I2C_Wait_Ack();
	data = API_I2C_ReceiveByte(0U);
	API_I2C_NAck();
	API_I2C_Stop();

	return data;
}

/* 读取芯片 ID。 */
uint8_t QMC_GetID(void)
{
	return QMC_ReadReg(QMC5883P_REG_CHIPID);
}

/*
 * 传感器初始化：
 * - CONTROL1 = 0xFF：连续模式 + 200Hz（沿用用户提供配置）
 * - CONTROL2 = 0x01：保持默认软复位/自检相关位配置
 */
void QMC_Init(void)
{
	QMC_WriteReg(QMC5883P_REG_CONTROL1, 0xFFU);
	QMC_WriteReg(QMC5883P_REG_CONTROL2, 0x01U);

#if (QMC_CAL_ENABLE == 1U)
	QMC_CalibBegin();
#endif
}

/* 读取三轴原始磁场数据 — 一次 I2C 突发读 6 字节保证数据一致性。 */
void QMC_GetData(int16_t *magX, int16_t *magY, int16_t *magZ)
{
	uint8_t buf[6];

	QMC_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(QMC5883P_I2C_ADDR_W);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(QMC5883P_REG_XOUT_L);   /* 起始寄存器，芯片自动递增 */
	API_I2C_Wait_Ack();

	API_I2C_Start();                            /* Repeated START */
	API_I2C_SendByte((uint8_t)(QMC5883P_I2C_ADDR_W | 0x01U));
	API_I2C_Wait_Ack();

	buf[0] = API_I2C_ReceiveByte(1U);           /* X_L, ACK  */
	buf[1] = API_I2C_ReceiveByte(1U);           /* X_H, ACK  */
	buf[2] = API_I2C_ReceiveByte(1U);           /* Y_L, ACK  */
	buf[3] = API_I2C_ReceiveByte(1U);           /* Y_H, ACK  */
	buf[4] = API_I2C_ReceiveByte(1U);           /* Z_L, ACK  */
	buf[5] = API_I2C_ReceiveByte(0U);           /* Z_H, NACK */
	API_I2C_Stop();

	*magX = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	*magY = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
	*magZ = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
}

/* =========================================================================
 * 磁力计校准 — 采集 min/max 计算 offset 和 scale
 *
 * 原理:
 *   无干扰时磁力计 XY 平面应是一个以原点为中心的圆。
 *   硬铁干扰把圆心平移 → offset = (max+min)/2
 *   软铁干扰把圆压成椭圆 → scale 把长短轴拉成等半径
 *
 * 只需校准一次，除非改动硬件布局（加/拆设备、换电机位置）。
 * 校准参数保存在 QMC5883P.h 宏中，断电不丢失。
 *
 * 校准流程:
 *   1. QMC_CAL_ENABLE 设为 1，烧录，把主循环里 QMC_Data() 移到 if 外面全速调用
 *   2. LED3 亮 = 校准进行中，水平旋转飞行器（覆盖 360 度）
 *   3. LED3 灭 = 校准完成，串口打印结果
 *   4. 把打印的四个值填入 QMC5883P.h 的宏
 *   5. QMC_CAL_ENABLE 改回 0，重新烧录
 * ========================================================================= */
#if (QMC_CAL_ENABLE == 1U)

#define QMC_CAL_DURATION_SEC    30U

extern uint32_t Timer_Bsp_t;   /* TIM2 维护的秒计数器 */

static int16_t  s_cal_min_x, s_cal_max_x;
static int16_t  s_cal_min_y, s_cal_max_y;
static uint16_t s_cal_count;
static uint32_t s_cal_start_s;
static uint8_t  s_cal_done = 0U;

void QMC_CalibBegin(void)
{
	s_cal_min_x   =  32767;
	s_cal_max_x   = -32768;
	s_cal_min_y   =  32767;
	s_cal_max_y   = -32768;
	s_cal_count    = 0U;
	s_cal_done     = 0U;
	s_cal_start_s  = Timer_Bsp_t;

	LED_Control(LED3, LED_HIGH);
}

void QMC_CalibSample(int16_t x, int16_t y)
{
	if (s_cal_done) return;

	if (x < s_cal_min_x) s_cal_min_x = x;
	if (x > s_cal_max_x) s_cal_max_x = x;
	if (y < s_cal_min_y) s_cal_min_y = y;
	if (y > s_cal_max_y) s_cal_max_y = y;
	s_cal_count++;

	if ((Timer_Bsp_t - s_cal_start_s) >= QMC_CAL_DURATION_SEC)
	{
		LED_Control(LED3, LED_LOW);
		s_cal_done = 1U;
		QMC_CalibEnd();
	}
}

uint8_t QMC_CalibIsDone(void)
{
	return s_cal_done;
}

void QMC_CalibEnd(void)
{
	int16_t offset_x, offset_y;
	float   range_x, range_y, avg_range, scale_x, scale_y;

	offset_x = (int16_t)(((int32_t)s_cal_max_x + (int32_t)s_cal_min_x) / 2);
	offset_y = (int16_t)(((int32_t)s_cal_max_y + (int32_t)s_cal_min_y) / 2);
	range_x  = (float)(s_cal_max_x - s_cal_min_x) * 0.5f;
	range_y  = (float)(s_cal_max_y - s_cal_min_y) * 0.5f;

	if (range_x < 1.0f) range_x = 1.0f;
	if (range_y < 1.0f) range_y = 1.0f;

	avg_range = (range_x + range_y) * 0.5f;
	scale_x   = avg_range / range_x;
	scale_y   = avg_range / range_y;

	usart_printf(USART1, "\r\n[CAL] ox=%d oy=%d sx=%.4f sy=%.4f\r\n",
		(int)offset_x, (int)offset_y,
		(double)scale_x, (double)scale_y);
}

#endif /* QMC_CAL_ENABLE */

/* 计算 XY 平面角度并返回。 */
float QMC_Data(void)
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	float   cal_x, cal_y;

	static float Angle_XY_temp = 0.0f;

	QMC_GetData(&x, &y, &z);
	(void)z;

#if (QMC_CAL_ENABLE == 1U)
	QMC_CalibSample(x, y);
#endif

	/* 硬铁 + 软铁校准 */
	cal_x = ((float)x - (float)QMC_CAL_OFFSET_X) * QMC_CAL_SCALE_X;
	cal_y = ((float)y - (float)QMC_CAL_OFFSET_Y) * QMC_CAL_SCALE_Y;

	/* atan2f 输出 [-180,180]，加 180 映射到 [0,360]，取反匹配指南针方向 */
	Angle_XY_temp = -(atan2f(cal_y, cal_x) * 57.2957795f) + 180.0f;
	Angle_XY = Angle_XY_temp;
	return Angle_XY_temp;
}
