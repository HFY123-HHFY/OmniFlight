#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>

#include "API_SPI.h"
#include "BusRate.h"
#include "NRF24L01_Define.h"

/*
 * NRF24L01 驱动说明：
 * 1) 标准 SPI 时序由 My_SPI 提供；
 * 2) 模块独有控制脚 CE 由注册层单独下发；
 * 3) 当前实现采用查询方式收发，不引入 IRQ 中断脚。
 */

/* NRF24L01 使用固定 5 字节地址与 32 字节包长。 */
#define NRF24L01_ADDR_WIDTH      5U
#define NRF24L01_TX_PACKET_WIDTH 32U
#define NRF24L01_RX_PACKET_WIDTH 32U

/* NRF24L01 模块专有控制脚（仅 CE，IRQ 暂不引入）。 */
typedef struct
{
	void *cePort;
	uint16_t cePin;
} NRF24L01_CtrlConfig_t;

/* NRF24L01 总线与速率: 统一在 SYSTEM/BusRate.h 集中配置 */

/* 发送地址与发送数据包。 */
extern uint8_t NRF24L01_TxAddress[NRF24L01_ADDR_WIDTH];
extern uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];

/* 接收地址与接收数据包。 */
extern uint8_t NRF24L01_RxAddress[NRF24L01_ADDR_WIDTH];
extern uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];

/* 遥控链路状态：收到有效遥控数据包时为 1，超时未收到时为 0。 */
extern volatile uint8_t NRF24L01_Linked;

/* 注册 NRF24L01 专有控制脚（CE）。 */
void NRF24L01_RegisterCtrl(const NRF24L01_CtrlConfig_t *configTable, uint8_t count);

/* 指令实现：寄存器与载荷操作。 */
uint8_t NRF24L01_ReadReg(uint8_t regAddress);
void NRF24L01_ReadRegs(uint8_t regAddress, uint8_t *dataArray, uint8_t count);
void NRF24L01_WriteReg(uint8_t regAddress, uint8_t data);
void NRF24L01_WriteRegs(uint8_t regAddress, const uint8_t *dataArray, uint8_t count);
void NRF24L01_WriteU16LE(uint8_t *buf, uint8_t offset, uint16_t value);
void NRF24L01_ReadRxPayload(uint8_t *dataArray, uint8_t count);
void NRF24L01_WriteTxPayload(const uint8_t *dataArray, uint8_t count);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);
uint8_t NRF24L01_ReadStatus(void);

/* 功能接口：模式切换、初始化、收发。 */
void NRF24L01_PowerDown(void);
void NRF24L01_StandbyI(void);
void NRF24L01_Rx(void);
void NRF24L01_Tx(void);
void NRF24L01_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);
void NRF24L01_UpdateRxAddress(void);

/* 最小回环读写测试：校验寄存器读写链路是否正常。 */
void App_NRF24L01_TestOnce(void);

/*
 * 遥控数据包约定（32 字节，前 3 字节为控制指令）：
 *   RxPacket[0] = Key ：1=解锁（手动油门） 2=锁定停机 3=定高（固定基准 400 + Altitude_Stick_Input 控高度）
 *   RxPacket[1] = 油门 0~250 → speed_temp（非回中摇杆，仅 Key==1 手动状态使用）
 *   RxPacket[2] = Altitude_Stick_Input 回中摇杆（-100~100），Key==3 定高时控高度
 */
extern volatile int8_t Altitude_Stick_Input; // 定高环摇杆输入（-100~100）
extern volatile int8_t Position_XY_Stick_Input; // 定点环摇杆输入（-100~100）

/* 和遥控器交换数据 */
void NRF24L01_RX_Data(void);   /* 接收数据包：解析遥控指令，Mode==1 置回传请求 */
void NRF24L01_TX_Data(void);   /* 发送数据包：请求有效时组包发送遥测 */

#endif /* __NRF24L01_H */
