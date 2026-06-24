#include "BMP280.h"

#include <math.h>

#include "API_I2C.h"
#include "Delay.h"
#include "LED.h"
#include "KEY.h"

/* 芯片自发热温升 (℃) */
#define BMP280_SELF_HEATING_OFFSET 8.0f
/* 地面气压 EMA 跟踪系数 */
#define BMP280_GND_EMA_ALPHA      0.02f

/* 最新一次计算得到的海拔高度（m，相对地面，0 为钳位下限）。 */
float alt = 0.0f;
/* 最新测量的大气气压（hPa），供串口打印/遥测。 */
float bmp_press = 0.0f;
/* 最新测量的芯片温度（℃），仅供串口观察，不参与高度计算（自发热偏高 ~10℃）。 */
float bmp_temp  = 0.0f;

/*
 * 地面基准（由 BMP280_ZeroAltitude() 写入）：
 *   ground_pressure    — 地面平均气压（hPa），高度公式的 P0
 *   ground_temperature — 地面平均温度（℃），高度公式的 T
 *
 * 初始值仅为 fallback，校准后即被实际采集值覆盖。
 * 高度公式用地面温度替代实测温度，避免芯片自发热导致高度漂移。
 */
static float ground_pressure    = 1013.25f;
static float ground_temperature = 27.0f;

/* BMP280 气压/温度过采样与工作模式配置。 */
#define BMP280_PRESSURE_OSR      (BMP280_OVERSAMP_8X)
#define BMP280_TEMPERATURE_OSR   (BMP280_OVERSAMP_16X)
#define BMP280_MODE              ((BMP280_PRESSURE_OSR << 2) | (BMP280_TEMPERATURE_OSR << 5) | BMP280_NORMAL_MODE)

typedef struct
{
	/* 温度补偿参数（来自芯片 NVM 校准区）。 */
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	/* 气压补偿参数（来自芯片 NVM 校准区）。 */
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	/* 中间变量：温度补偿结果，供气压补偿复用。 */
	int32_t t_fine;
} bmp280Calib_t;

/* BMP280 校准参数缓存。 */
static bmp280Calib_t bmp280Cal;

/* 芯片 ID 缓存。 */
static uint8_t bmp280ID = 0U;
/* 初始化标记，避免重复配置。 */
static bool isInit = false;
/* 20-bit 原始气压 ADC 数据。 */
static int32_t bmp280RawPressure = 0;
/* 20-bit 原始温度 ADC 数据。 */
static int32_t bmp280RawTemperature = 0;

/* 读取一帧原始温度/气压数据。 */
static void bmp280GetPressure(void);
/* 气压值转换为海拔高度（相对地面，钳位 >= 0）。 */
static float bmp280PressureToAltitude(float pressure, float temperature);
/* 温度补偿（输出单位：0.01 摄氏度）。 */
static int32_t bmp280CompensateT(int32_t adcT);
/* 气压补偿（输出单位：Q24.8 Pa）。 */
static uint32_t bmp280CompensateP(int32_t adcP);


/* 选择 I2C1，设 BMP280 I2C 速率为 400kHz。 */
static void BMP280_SelectI2CSpeed(void)
{
	API_I2C_SelectBus(BMP280_I2C_BUS);
	API_I2C_SetSpeed(BMP280_I2C_SPEED);
}

/*
 * 读取单个寄存器。
 * devaddr: 8-bit I2C 地址（写地址）
 * addr   : 目标寄存器地址
 * return : 读取到的 1 字节数据
 */
uint8_t iicDevReadByte(uint8_t devaddr, uint8_t addr)
{
	uint8_t temp;

	BMP280_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(devaddr);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(addr);
	API_I2C_Wait_Ack();

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(devaddr | 0x01U));
	API_I2C_Wait_Ack();
	temp = API_I2C_ReceiveByte(0U);
	API_I2C_Stop();

	return temp;
}

/*
 * 连续读取多个寄存器。
 * devaddr: 8-bit I2C 地址（写地址）
 * addr   : 起始寄存器地址
 * len    : 读取长度
 * rbuf   : 输出缓存
 */
void iicDevRead(uint8_t devaddr, uint8_t addr, uint8_t len, uint8_t *rbuf)
{
	uint8_t i;

	BMP280_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(devaddr);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(addr);
	API_I2C_Wait_Ack();

	API_I2C_Start();
	API_I2C_SendByte((uint8_t)(devaddr | 0x01U));
	API_I2C_Wait_Ack();

	for (i = 0U; i < len; i++)
	{
		if (i == (uint8_t)(len - 1U))
		{
			rbuf[i] = API_I2C_ReceiveByte(0U);
		}
		else
		{
			rbuf[i] = API_I2C_ReceiveByte(1U);
		}
	}

	API_I2C_Stop();
}

/*
 * 写入单个寄存器。
 * devaddr: 8-bit I2C 地址（写地址）
 * addr   : 目标寄存器地址
 * data   : 写入数据
 */
void iicDevWriteByte(uint8_t devaddr, uint8_t addr, uint8_t data)
{
	BMP280_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(devaddr);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(addr);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(data);
	API_I2C_Wait_Ack();
	API_I2C_Stop();
}

/*
 * 连续写入多个寄存器。
 * devaddr: 8-bit I2C 地址（写地址）
 * addr   : 起始寄存器地址
 * len    : 写入长度
 * wbuf   : 输入缓存
 */
void iicDevWrite(uint8_t devaddr, uint8_t addr, uint8_t len, uint8_t *wbuf)
{
	uint8_t i;

	BMP280_SelectI2CSpeed();

	API_I2C_Start();
	API_I2C_SendByte(devaddr);
	API_I2C_Wait_Ack();
	API_I2C_SendByte(addr);
	API_I2C_Wait_Ack();

	for (i = 0U; i < len; i++)
	{
		API_I2C_SendByte(wbuf[i]);
		API_I2C_Wait_Ack();
	}

	API_I2C_Stop();
}

/*
 * BMP280 初始化 + 自动地面归零。
 *
 * 流程：
 *   1) 读取并校验芯片 ID
 *   2) 读取 24 字节校准参数
 *   3) 配置过采样 + Normal 模式 + IIR 系数 8
 *   4) 自动执行 5 秒地面归零校准（传感器热稳定 + 采集基准气压/温度）
 *
 * 注意：校准期间飞行器必须保持静止！
 */
bool BMP280Init(void)
{
	if (isInit)
	{
		return true;
	}

	bmp280ID = iicDevReadByte(BMP280_ADDR, BMP280_CHIP_ID);
	if (bmp280ID != BMP280_DEFAULT_CHIP_ID)
	{
		return false;
	}

	iicDevRead(BMP280_ADDR,
			   BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,
			   BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH,
			   (uint8_t *)&bmp280Cal);

	iicDevWriteByte(BMP280_ADDR, BMP280_CTRL_MEAS_REG, BMP280_MODE);
	iicDevWriteByte(BMP280_ADDR, BMP280_CONFIG_REG, (uint8_t)((0U << 5) | (3U << 2) | 0U));

	isInit = true;

	/* 自动执行地面归零校准（5 秒），飞行器必须保持静止！ */
	BMP280_ZeroAltitude(5000U);
	return true;
}

/* 从数据寄存器读取一帧 6 字节原始数据并拆包为 20-bit ADC 值。 */
static void bmp280GetPressure(void)
{
	uint8_t data[BMP280_DATA_FRAME_SIZE];

	iicDevRead(BMP280_ADDR, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, data);
	bmp280RawPressure = (int32_t)((((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4)));
	bmp280RawTemperature = (int32_t)((((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4)));
}

/*
 * 温度补偿（Bosch 推荐公式）。
 * adcT: 原始温度 ADC
 * return: 温度值，单位 0.01 摄氏度
 */
static int32_t bmp280CompensateT(int32_t adcT)
{
	int32_t var1;
	int32_t var2;
	int32_t t;

	var1 = ((((adcT >> 3) - ((int32_t)bmp280Cal.dig_T1 << 1)))*((int32_t)bmp280Cal.dig_T2)) >> 11;
	var2 = (((((adcT >> 4) - ((int32_t)bmp280Cal.dig_T1)) * ((adcT >> 4) - ((int32_t)bmp280Cal.dig_T1))) >> 12) * ((int32_t)bmp280Cal.dig_T3)) >> 14;
	bmp280Cal.t_fine = var1 + var2;

	t = (bmp280Cal.t_fine * 5 + 128) >> 8;
	return t;
}

/*
 * 气压补偿（Bosch 推荐公式，64-bit 防溢出）。
 * adcP: 原始气压 ADC
 * return: Q24.8 格式的 Pa
 */
static uint32_t bmp280CompensateP(int32_t adcP)
{
	int64_t var1;
	int64_t var2;
	int64_t p;

	var1 = ((int64_t)bmp280Cal.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bmp280Cal.dig_P6;
	var2 = var2 + ((var1 * (int64_t)bmp280Cal.dig_P5) << 17);
	var2 = var2 + (((int64_t)bmp280Cal.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)bmp280Cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280Cal.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)bmp280Cal.dig_P1)) >> 33;

	if (var1 == 0)
	{
		return 0U;
	}

	p = 1048576 - adcP;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)bmp280Cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)bmp280Cal.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280Cal.dig_P7) << 4);

	return (uint32_t)p;
}

/* 标准大气压换算指数 1/5.25588。 */
#define CONST_PF  0.1902630958f

/*
 * 气压 → 相对海拔（m）。
 *
 * 公式：h = ((P0 / P)^(1/5.25588) - 1) * (T + 273.15) / 0.0065
 *   P0 = ground_pressure  — 地面气压（未解锁时 EMA 跟踪）
 *   T  = 实测温度 - 自发热偏置 (8℃)
 *
 * 关键设计：
 *   - 使用实测温度减去自发热偏置，而非固定值。
 *   - 未解锁时 ground_pressure 持续 EMA 跟踪，消除芯片热漂移导致的高度不回零。
 *   - 解锁后 ground_pressure 冻结，保证飞行中高度参照稳定。
 *   - 输出钳位到 >= 0。
 */
static float bmp280PressureToAltitude(float pressure, float temperature)
{
	float h;
	float t_ambient;

	if (pressure > 0.0f && ground_pressure > 0.0f)
	{
		t_ambient = temperature - BMP280_SELF_HEATING_OFFSET;
		if (t_ambient < -40.0f) { t_ambient = -40.0f; }
		h = ((powf((ground_pressure / pressure), CONST_PF) - 1.0f) * (t_ambient + 273.15f)) / 0.0065f;
		if (h < 0.0f) { h = 0.0f; }
		return h;
	}

	return 0.0f;
}

/*
 * 地面高度归零：持续采集 duration_ms 毫秒，取气压和温度的平均值
 * 作为地面基准。
 *
 * 调用后 BMP_Data() 返回相对于该基准的高度（地面 ≈ 0m）。
 *
 * duration_ms: 校准时长（ms），Init 自动传入 5000（5 秒）
 *   - 前 1~2 秒传感器热稳定，后续采样为有效基准
 *   - 每 20ms 采一帧，5 秒共 ~250 帧，平均后噪声 < 0.05m
 */
void BMP280_ZeroAltitude(uint32_t duration_ms)
{
	float sum_p = 0.0f;
	float sum_t = 0.0f;
	float p, t, a;
	uint32_t elapsed;
	uint16_t cnt = 0U;

	for (elapsed = 0U; elapsed < duration_ms; elapsed += 20U)
	{
		BMP280GetData(&p, &t, &a);
		sum_p += p;
		sum_t += t;
		cnt++;
		Delay_ms(20U);   /* 等待传感器产生新数据（Normal 模式周期约 9ms） */
	}

	ground_pressure    = sum_p / (float)cnt;
	ground_temperature = sum_t / (float)cnt;
}

/*
 * 获取 BMP280 实测数据：
 * pressure   → 气压（hPa）
 * temperature→ 芯片温度（℃，仅供参考，高于环境温度 ~10℃）
 * asl        → 海拔（m，相对地面归零基准，钳位 >= 0）
 */
void BMP280GetData(float *pressure, float *temperature, float *asl)
{
	float t;
	float p;

	bmp280GetPressure();

	t = (float)bmp280CompensateT(bmp280RawTemperature) / 100.0f;
	p = (float)bmp280CompensateP(bmp280RawPressure) / 25600.0f;

	*pressure    = p;
	*temperature = t;
	*asl         = bmp280PressureToAltitude(p, t);

	/* 更新全局变量，供外部串口打印/遥测 */
	bmp_press    = p;
	bmp_temp     = t;
	alt          = *asl;
}

/*
 * 返回相对地面高度 (m)。
 *
 * 地面气压跟踪策略：
 *   未解锁 (Key!=1): 慢速 EMA 跟踪气压变化，补偿芯片热漂移
 *   解锁 (Key==1):  冻结 ground_pressure，保证飞行中高度参照稳定
 *   锁定瞬间: 快速重采地面气压 (2s)， alt 立即归零
 *
 * 注意：飞完降落后必须先锁定 (Key!=1)，等 2s alt 归零再重新解锁，
 * 否则 ground_pressure 停在旧值，放回原位 alt 不会归零。
 */
float BMP_Data(void)
{
	static uint8_t  prev_key       = 0U;
	static uint8_t  disarm_samples = 0U;
	static float    disarm_press_sum = 0.0f;
	float bmp280_press = 0.0f;
	float bmp280_temp  = 0.0f;
	float bmp280_asl   = 0.0f;

	BMP280GetData(&bmp280_press, &bmp280_temp, &bmp280_asl);

	/* Key 1->0 边沿：开始快速归零 (2s 采 40 帧) */
	if (prev_key == 1 && Key != 1)
	{
		disarm_samples = 0U;
		disarm_press_sum = 0.0f;
	}
	prev_key = Key;

	if (Key != 1)
	{
		if (disarm_samples < 40U)
		{
			/* 前 2 秒快速平均 */
			disarm_press_sum += bmp280_press;
			disarm_samples++;
			if (disarm_samples >= 40U)
			{
				ground_pressure = disarm_press_sum / 40.0f;
			}
		}
		else
		{
			/* 归零完成后恢复慢速 EMA 跟踪 */
			ground_pressure += BMP280_GND_EMA_ALPHA * (bmp280_press - ground_pressure);
		}
	}
	/* Key==1: 冻结 ground_pressure，不更新 */

	return bmp280_asl;
}
