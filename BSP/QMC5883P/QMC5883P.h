#ifndef __QMC5883P_H
#define __QMC5883P_H

#include <stdint.h>
#include "API_I2C.h"
#include "BusRate.h"

#ifdef __cplusplus
extern "C" {
#endif

/* QMC5883P I2C 8-bit 地址（写地址） */
#define QMC5883P_I2C_ADDR_W      0x58U

/* QMC5883P 总线与速率: 统一在 SYSTEM/BusRate.h 集中配置 */

/* 寄存器定义 */
#define QMC5883P_REG_CHIPID       0x00U
#define QMC5883P_REG_XOUT_L       0x01U
#define QMC5883P_REG_XOUT_H       0x02U
#define QMC5883P_REG_YOUT_L       0x03U
#define QMC5883P_REG_YOUT_H       0x04U
#define QMC5883P_REG_ZOUT_L       0x05U
#define QMC5883P_REG_ZOUT_H       0x06U
#define QMC5883P_REG_STATUS       0x09U
#define QMC5883P_REG_CONTROL1     0x0AU
#define QMC5883P_REG_CONTROL2     0x0BU

/* 最近一次计算得到的 XY 平面航向角（单位：度，范围 0~360） */
extern float Angle_XY;

/* =========================================================================
 * 磁力计校准
 *
 * 硬铁偏移 (offset)：PCB 上永磁体/大电流走线造成的固定偏置
 * 软铁缩放 (scale)：铁磁材料造成的椭圆畸变
 *
 * 校准流程:
 *   1. 取消 QMC_CAL_ENABLE 的注释
 *   2. 水平旋转飞行器 30 秒
 *   3. 串口输出 "QMC_CAL: offset_x=... offset_y=... scale_x=... scale_y=..."
 *   4. 把输出的四个值填入下面的宏
 *   5. 重新注释 QMC_CAL_ENABLE
 * ========================================================================= */

/* 设为 1 开启校准模式（采集 min/max），校准完成后改回 0 */
#define QMC_CAL_ENABLE          0U

/* 校准参数 — 5 次校准取平均 */
#define QMC_CAL_OFFSET_X        -2209
#define QMC_CAL_OFFSET_Y        2640
#define QMC_CAL_SCALE_X         1.0182f
#define QMC_CAL_SCALE_Y         0.9828f

/* 校准 API */
void    QMC_CalibBegin(void);
void    QMC_CalibSample(int16_t x, int16_t y);
void    QMC_CalibEnd(void);
uint8_t QMC_CalibIsDone(void);

/*
 * 初始化 QMC5883P：
 * - 配置连续测量模式
 * - 设置输出数据率与基本工作参数
 */
void QMC_Init(void);

/* 读取芯片 ID（寄存器 0x00）。 */
uint8_t QMC_GetID(void);

/* 读取三轴原始磁场数据（单位为原始计数值）。 */
void QMC_GetData(int16_t *magX, int16_t *magY, int16_t *magZ);

/*
 * 计算 XY 平面角度（0~360 度）：
 * - 使用 atan2f(y, x)
 * - 结果同步写入全局变量 Angle_XY
 */
float QMC_Data(void);

#ifdef __cplusplus
}
#endif


#endif /* __QMC5883P_H */
