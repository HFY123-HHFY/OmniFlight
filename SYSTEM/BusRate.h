#ifndef __BUS_RATE_H
#define __BUS_RATE_H

#include "API_I2C.h"
#include "API_SPI.h"

/*
 * BusRate.h — 总线统一配置中心
 *
 * 负责：I2C 模式切换、总线选择、速率档位
 * 规则：改 I2C 模式/总线/速率只改这一个文件
 */

/* ── I2C 模式（三平台共用枚举）── */
#define I2C_MODE_SOFT   1U
#define I2C_MODE_HARD   2U

/* ================================================================
 *  F103
 * ================================================================ */
#if (ENROLL_MCU_TARGET == ENROLL_MCU_F103)

/* I2C 模式 — F103 只有软件 */
#define I2C_MODE          I2C_MODE_SOFT

/* 总线选择 */
#define OLED_I2C_BUS       API_I2C2
#define OLED_SPI_BUS       API_SPI1
#define MPU6050_I2C_BUS    API_I2C1
#define NRF24L01_SPI_BUS   API_SPI2

/* 速率 */
#define OLED_I2C_SPEED      API_I2C_SPEED_400K
#define MPU6050_I2C_SPEED   API_I2C_SPEED_400K
#define OLED_SPI_SPEED      API_SPI_SPEED_1M
#define NRF24L01_SPI_SPEED  API_SPI_SPEED_1M

/* ================================================================
 *  F407
 * ================================================================ */
#elif (ENROLL_MCU_TARGET == ENROLL_MCU_F407)

/*
 * I2C 模式：软硬件二选一，改下面这行
 *   I2C_MODE_SOFT — GPIO bit-bang
 *   I2C_MODE_HARD — 片上 I2C1
 */
#define I2C_MODE          I2C_MODE_HARD

/* 总线选择 */
#define MPU6050_I2C_BUS    API_I2C1
#define QMC5883P_I2C_BUS   API_I2C1
#define BMP280_I2C_BUS     API_I2C1
#define OLED_I2C_BUS       API_I2C2
#define OLED_SPI_BUS       API_SPI1
#define NRF24L01_SPI_BUS   API_SPI1

/* 速率 — 软硬件统一，改这里生效 */
#define MPU6050_I2C_SPEED   API_I2C_SPEED_400K
#define QMC5883P_I2C_SPEED  API_I2C_SPEED_400K
#define BMP280_I2C_SPEED    API_I2C_SPEED_400K
#define OLED_I2C_SPEED      API_I2C_SPEED_400K
#define OLED_SPI_SPEED      API_SPI_SPEED_5M
#define NRF24L01_SPI_SPEED  API_SPI_SPEED_5M

/* ================================================================
 *  G3507
 * ================================================================ */
#elif (ENROLL_MCU_TARGET == ENROLL_MCU_G3507)

/* I2C 模式 — G3507 只有软件 */
#define I2C_MODE          I2C_MODE_SOFT

/* 总线选择 */
#define OLED_I2C_BUS       API_I2C2
#define OLED_SPI_BUS       API_SPI1
#define MPU6050_I2C_BUS    API_I2C1

/* 速率 */
#define OLED_I2C_SPEED      API_I2C_SPEED_200K
#define MPU6050_I2C_SPEED   API_I2C_SPEED_400K
#define OLED_SPI_SPEED      API_SPI_SPEED_5M

#else
#error "Unsupported ENROLL_MCU_TARGET for bus rate profile."
#endif

#endif /* __BUS_RATE_H */
