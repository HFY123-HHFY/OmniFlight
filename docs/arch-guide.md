# OmniFlight 工程架构深度解析

> 本文档用于 AI 上下文初始化，新对话加载此文档即可快速恢复对项目架构的完整认知。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| **名称** | OmniFlight — 四轴飞控 |
| **基于框架** | [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) |
| **主控** | STM32F407VET6 (Cortex-M4 + FPU, 168MHz, 512KB Flash, 128KB RAM) |
| **构建工具** | CMake + GCC ARM Embedded + OpenOCD |
| **IDE** | VS Code (主) + Keil MDK (保留) |
| **默认 MCU** | `ENROLL_MCU_F407` (定义于 Enroll/Enroll.h) |
| **分支** | `main` (裸机飞控主线) |

---

## 2. 分层架构

```
┌────────────────────────────────────────┐
│  A_Entry/main.c  程序入口               │  飞控主循环
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  app/           应用层 — 飞控核心算法   │
│  Control/       串级PID + 混控 + 电机   │
│  Control_Task/  中断回调 + 任务标志     │
│  PID/           PID 控制器              │
│  Filter/        低通/互补滤波器         │
│  My_Usart/      串口管理 + printf       │
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  BSP/           板级支持层              │
│  MPU6050/ QMC5883P/ BMP280/            │  传感器驱动
│  NRF24L01/      2.4G 无线              │
│  Dshot/         DShot300 油门协议       │
│  Motor/         电机混控               │
│  Buzzer/        蜂鸣器                 │
│  LED/ KEY/ OLED/ TB6612/               │
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  Enroll/        注册层 (★核心特色)      │  硬件资源注册中心
│  407_hw_config.h  F407 板级映射        │  X-Macro 编译期展开
│  Enroll.c        注册门面函数          │
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  API/           片内外设抽象接口层       │
│  gpio/adc/pwm/tim/usart/exti           │  统一接口，屏蔽芯片差异
│  API_I2C/ API_SPI/  软件总线协议层     │  条件编译分发到 Core
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  Core/          芯片底层实现            │
│  STM32F407/     f407_gpio/pwm/tim/...  │  直接寄存器操作
│  STM32F103/ MSPM0G3507/ (保留)         │  + f407_dma (DMA 抽象)
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  Drivers/       驱动资源层              │  CMSIS + 启动文件
│  SYSTEM/        系统层                  │  sys/Delay/BusRate/IrqPriority
└────────────────────────────────────────┘
```

---

## 3. 当前开发 MCU

| MCU | 状态 |
|-----|:---:|
| **STM32F407VET6** | ★ 主力开发 |
| STM32F103C8T6 | 架构保留 |
| MSPM0G3507 | 架构保留 |

---

## 4. F407 飞控板引脚配置

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| I2C SCL | PB8 | 软件 I2C | MPU/QMC/BMP 共用 |
| I2C SDA | PB9 | 软件 I2C | |
| MPU6050 INT | PE7 | EXTI | DMP 数据就绪 |
| NRF SCK | PA5 | SPI1 | 软件 SPI |
| NRF MOSI | PA7 | SPI1 | |
| NRF MISO | PA6 | SPI1 | |
| NRF CS | PC4 | GPIO | |
| NRF CE | PC5 | GPIO | Enroll 注册 |
| 电机1 | PE9 | TIM1 CH1 | DShot300 |
| 电机2 | PE11 | TIM1 CH2 | |
| 电机3 | PE13 | TIM1 CH3 | |
| 电机4 | PE14 | TIM1 CH4 | |
| 蜂鸣器 | PB1 | TIM3 CH4 | 2700Hz PWM |
| USART1 TX/RX | PA9/PA10 | USART1 | 调试 115200 |
| USART3 TX/RX | PD8/PD9 | USART3 | 无线串口 |
| LED1/2/3 | PE2/PE3/PE4 | GPIO | 绿/红/蓝 |

---

## 5. 飞控核心设计

### 5.1 串级 PID

```
目标角度 (遥控器) → 角度环 PID (外环 250Hz) → 角速度环 PID (内环 500Hz) → 混控 → DShot
```

### 5.2 DShot300 油门协议

- TIM1 配置为 300kHz PWM 基波（ARR=560, PSC=1）
- DMA2 Stream5 burst 模式，每次 TIM1 溢出自动更新 CCR1~CCR4
- 4 路电机通过 X-Macro 映射到物理通道：CH1←m3, CH2←m1, CH3←m2, CH4←m4
- 油门范围：48~2047（0 停转，1~47 为保留命令区）
- **重要**：电调需要从最低油门（48）逐步递增，不可直接跳到大油门值

### 5.3 中断回调架构

- `Control_Task1_Callback` → TIM3 1ms → pid_task_flag (2ms/500Hz)
- `Control_Task2_Callback` → TIM2 1ms → print_task_flag (100ms)
- `Control_Task_USART_Callback` → USART1/3 → TX 队列排空 + RX 接收

### 5.4 控制任务标志位

- `pid_task_flag` — 500Hz 姿态环触发
- `print_task_flag` — 100ms 串口打印触发
- NRF24L01 在主循环轮询（无中断）

---

## 6. 核心设计模式

### 6.1 注册层 (Enroll + X-Macro)

```c
// 407_hw_config.h 定义映射宏
#define HW_DSHOT_MOTOR_MAP(X) \
    X(1U, GPIOE, GPIO_Pin_9)   \
    X(2U, GPIOE, GPIO_Pin_11)  \
    ...

// Dshot.c 编译期展开
HW_DSHOT_MOTOR_MAP(DSHOT_CFG_PIN)
// → F407_PWM_ConfigPin(GPIOE, Pin_9, 1)
// → F407_PWM_ConfigPin(GPIOE, Pin_11, 1)
// ...

// 切换 MCU 只需提供新的 xxx_hw_config.h
```

### 6.2 USART 异步 TX 防护

- `usart_send_byte_async` 发送前检查 `asyncReady` 标志
- 标志默认 0（安全），注册中断回调时通过 weak 函数钩子自动置 1
- 未注册回调时退化到阻塞发送，不会崩溃
- `Enroll_USART_RegisterIrqHandler` 必须调用，否则 TXE 中断无限循环

### 6.3 软件总线双分层

```
API/API_I2C/API_I2C.c   ← 协议逻辑 (平台无关)
    ↓ soft_i2c_hal.h 桥接
Core/STM32F407/f407_soft_i2c/  ← GPIO 翻转+延时 (平台相关)
```

---

## 7. 中断优先级 (IrqPriority.h)

| 优先级 | 中断源 | 理由 |
|:---:|------|------|
| 0 | SysTick | 系统心跳基准 |
| 1 | TIM3 (API_TIM1) | 1ms 控制节拍，PID/混控心脏 |
| 2 | MPU6050 EXTI | 姿态数据，串级外环输入 |
| 3 | TIM2 (API_TIM2) | printf / 时间戳 |
| 4 | USART1/3 | 通信（丢包可重传） |

---

## 8. BSP 模块清单

| 模块 | 依赖层 | 说明 |
|------|--------|------|
| LED | API_GPIO | Enroll 注册 |
| MPU6050 | API_I2C + EXTI | DMP 姿态解算 |
| QMC5883P | API_I2C | 地磁航向 |
| BMP280 | API_I2C | 气压高度 |
| NRF24L01 | API_SPI + Enroll CE | 2.4G 遥控遥测 |
| Dshot | Core f407_pwm + f407_dma | DShot300 油门 |
| Motor | Dshot + Control | 混控反饱和 |
| Buzzer | API_PWM | 无源蜂鸣器 |

---

## 9. 已知注意事项

1. **USART3 AF 修复**：`API_USART_GetAfNum` 从 AF8 改为 AF7（STM32F407 所有 USART 都是 AF7）
2. **NRF24L01 CE 注册**：`Enroll_NRF24L01_Register()` 必须在 `NRF24L01_Init()` 前调用
3. **DShot 电调**：油门值需从最低（48）逐步递增，电调才响应
4. **printf 异步 TX**：必须注册 USART 中断回调，否则 TX 队列不会排空
5. **F407 以外平台**：f407_dma、Dshot、Motor、Buzzer 仅在 F407 编译

---

## 10. 快速恢复认知

1. [README.md](README.md) — 项目概览
2. 本文档 — 架构全貌
3. [A_Entry/main.c](A_Entry/main.c) — 飞控初始化 → 主循环
4. [Enroll/407_hw_config.h](Enroll/407_hw_config.h) — F407 板级映射
5. [BSP/Dshot/Dshot.c](BSP/Dshot/Dshot.c) — DShot300 协议
6. [SYSTEM/IrqPriority.h](SYSTEM/IrqPriority.h) — 中断优先级
7. [app/Control_Task/Control_Task.c](app/Control_Task/Control_Task.c) — 中断回调
8. [CMakeLists.txt](CMakeLists.txt) — 构建入口
