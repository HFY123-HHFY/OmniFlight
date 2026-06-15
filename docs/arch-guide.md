# OmniFlight 工程架构深度解析

> 本文档旨在帮助在多个 Claude Code 对话中快速恢复对工程架构的完整认知。
> 每次重新开启对话后，Claude Code 只需阅读本文档即可快速理解项目分层、设计原则与编码约定。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| **名称** | OmniFlight — 四轴飞控 |
| **基于框架** | [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) 分层架构 |
| **定位** | 基于 OmniLayer 架构的 STM32F407 四轴飞控项目 |
| **主控** | STM32F407VET6 (Cortex-M4 + FPU, 168MHz) |
| **构建工具** | CMake + GCC ARM Embedded + OpenOCD |
| **IDE** | VS Code (主) + Keil MDK (保留兼容) |
| **默认 MCU** | `ENROLL_MCU_F407` (定义于 Enroll/Enroll.h) |
| **分支策略** | `main` (裸机飞控主线) |

---

## 2. 当前开发目标 MCU

| MCU | 架构 | 内核 | 当前状态 |
|-----|------|------|:---:|
| **STM32F407VET6** | ARM | Cortex-M4 + FPU | ★ 主力开发 |
| STM32F103C8T6 | ARM | Cortex-M3 | 架构保留，暂不开发 |
| TI MSPM0G3507 | ARM | Cortex-M0+ | 架构保留，暂不开发 |

三条 MCU 产品线在同一个 CMake 工程中维护，通过 `ENROLL_MCU_TARGET` 宏 (0/1/2) 在编译期切换。
当前默认值为 `ENROLL_MCU_F407`。F103 和 G3507 的文件架构保留，不影响 F407 的日常开发与编译。

---

## 3. 分层架构总览

```
┌────────────────────────────────────────────┐
│  A_Entry/      程序入口                     │  飞控主循环
│  main.c        初始化 → 注册 → 传感器 → 控制│  唯一入口
└────────────────────────────────────────────┘
              ↓ 调用
┌────────────────────────────────────────────┐
│  app/          应用层 — 飞控核心算法        │
│  - Control/    串级 PID + 混控 + 电机加载   │
│  - Control_Task/ 中断回调 + 任务标志位      │
│  - PID/        PID 控制器                   │
│  - Filter/     低通/互补滤波器              │
│  - My_Usart/   串口打印管理                 │
└────────────────────────────────────────────┘
              ↓ 调用
┌────────────────────────────────────────────┐
│  BSP/          板级支持层                   │  传感器与器件驱动
│  - LED/KEY     GPIO 控制型外设              │
│  - MPU6050     六轴陀螺仪 + DMP 姿态解算    │
│  - QMC5883P    三轴磁力计                   │
│  - BMP280      气压/温度传感器              │
│  - NRF24L01    2.4G 无线收发模块            │
│  - OLED        显示屏 (I2C/SPI 双模式)      │
│  - TB6612      电机驱动 (预留)              │
└────────────────────────────────────────────┘
              ↓ 依赖
┌────────────────────────────────────────────┐
│  Enroll/       注册层 (★核心特色)           │  硬件资源注册中心
│  - Enroll.h    对外接口 + 默认 MCU 宏       │
│  - Enroll.c    注册实现 (X-Macro 展开)      │
│  - Enroll_Internal.h  内部依赖              │
│  - xxx_hw_config.h  板级映射表 (3个MCU)      │
└────────────────────────────────────────────┘
              ↓ 绑定
┌────────────────────────────────────────────┐
│  API/          片内外设抽象接口层            │  统一接口，屏蔽芯片差异
│  inc/ + src/   gpio/adc/pwm/tim/usart/exti  │
│  API_I2C/      I2C 协议层 (平台无关)        │
│  API_SPI/      SPI 协议层 (平台无关)        │
│  通过 soft_xxx_hal 桥接到 Core 层            │
└────────────────────────────────────────────┘
              ↓ 分发
┌────────────────────────────────────────────┐
│  Core/         芯片底层实现                  │  按 MCU 分目录
│  STM32F103/    {src,inc}/f103_*.c,h         │
│  STM32F407/    {src,inc}/f407_*.c,h         │
│  MSPM0G3507/   {src,inc}/G3507_*.c,h        │
│  (每目录含: soft_i2c, soft_spi, cmake/linker)│
└────────────────────────────────────────────┘
              ↓ 基于
┌────────────────────────────────────────────┐
│  Drivers/      驱动资源层                   │  启动文件、CMSIS/HAL
│  Drivers_STM32F1/  - std_periph 启动        │
│  Drivers_STM32F4/  - std_periph 启动        │
│  Drivers_M0G3507/  - TI DriverLib + CMSIS   │
└────────────────────────────────────────────┘
              ↓ 基础
┌────────────────────────────────────────────┐
│  SYSTEM/       系统层                       │  系统配置与初始化
│  sys.c/h      系统初始化 / 中断分发         │
│  Delay.h      统一延时接口                  │
│  BusRate.h    软件总线选择+速率集中配置     │
│  IrqPriority.h 统一中断优先级管理           │
└────────────────────────────────────────────┘
```

---

## 4. 核心设计模式

### 4.1 注册层模式 (Enroll — 最重要的设计)

注册层的本质是**用编译期 X-Macro 将"逻辑外设 ID"映射到"物理引脚+硬件实例"**。

**数据流：**
```
407_hw_config.h (板级映射宏)
    ↓ 定义 HW_PWM_MAP(X) / HW_I2C_MAP(X) / HW_USART_MAP(X) 等
Enroll.c (X-Macro 展开)
    ↓ 展开为结构体数组
Enroll_xxx_Register() (门面函数)
    ↓ 传入结构体数组 + 计数
API/BSP 的 Register() / Init() 函数
    ↓ 写入内部管理数组
后续 API/BSP 用逻辑 ID 操作
```

**示例 — F407 的 PWM 注册：**

`407_hw_config.h` 定义映射宏：
```c
#define HW_PWM_MAP(X) \
    X(API_PWM_TIM1, API_PWM_CH1, API_PWM_CORE_TIM1, API_PWM_CORE_CH1, GPIOE, GPIO_Pin_9) \
    X(API_PWM_TIM1, API_PWM_CH2, API_PWM_CORE_TIM1, API_PWM_CORE_CH2, GPIOE, GPIO_Pin_11) \
    X(API_PWM_TIM1, API_PWM_CH3, API_PWM_CORE_TIM1, API_PWM_CORE_CH3, GPIOE, GPIO_Pin_13) \
    X(API_PWM_TIM1, API_PWM_CH4, API_PWM_CORE_TIM1, API_PWM_CORE_CH4, GPIOE, GPIO_Pin_14)
```

`Enroll.c` 用 X-Macro 展开为配置表：
```c
#define ENROLL_PWM_ITEM(logical_tim, logical_ch, core_tim, core_ch, port, pin) \
    { logical_tim, logical_ch, core_tim, core_ch, (void *)port, pin },
static const PWM_Config_t s_pwmTable[] = {
    HW_PWM_MAP(ENROLL_PWM_ITEM)
};
```

优势：切换 MCU 时只需提供新的 `xxx_hw_config.h`，Enroll.c 无需改动。

### 4.2 API 条件编译分发模式

API 层提供统一接口，内部通过 `#if ENROLL_MCU_TARGET` 分发：

```c
// API/inc/gpio.h — 接口声明
void API_GPIO_Write(void *port, uint32_t pin, uint8_t level);

// API/src/gpio.c — 实现分发
void API_GPIO_Write(void *port, uint32_t pin, uint8_t level) {
#if (ENROLL_MCU_TARGET == ENROLL_MCU_F103)
    F103_GPIO_Write(port, pin, level);
#elif (ENROLL_MCU_TARGET == ENROLL_MCU_F407)
    F407_GPIO_Write(port, pin, level);
#elif (ENROLL_MCU_TARGET == ENROLL_MCU_G3507)
    G3507_GPIO_Write(port, pin, level);
#endif
}
```

**注意：** 这是编译期多态，不是运行时虚表 — 一个固件只编译一个 MCU 目标。

### 4.3 两阶段初始化模式 (Register → Init)

飞控外设资源使用两阶段初始化：

```
Enroll_xxx_Register()  // 阶段1: 登记配置表 (填充内部数组)
        ↓
API_xxx_Init(id, ...)  // 阶段2: 激活硬件 (写寄存器、开启时钟)
```

这在 `main.c` 中体现得最明显：

```c
// 注册阶段
Enroll_LED_Register();
Enroll_USART_Register();
Enroll_PWM_Register();
Enroll_TIM_Register();
Enroll_I2C_Register();
Enroll_SPI_Register();

// 初始化阶段
API_USART_Init(API_USART1, 115200U);
API_PWM_Init(API_PWM_TIM1, 4000U - 1U, 840U - 1U); // 50Hz
API_TIM_Init(API_TIM1, 1U);   // PID 节拍 1ms
API_I2C_Init();
API_SPI_Init();

// 传感器初始化（依赖 I2C 已激活）
MPU_Init();
QMC_Init();
BMP280Init();
```

### 4.4 void *port 跨 MCU 抽象

由于不同 MCU 的 GPIO 端口类型不同（STM32: `GPIO_TypeDef *`），API 层统一用 `void *port` 传递端口指针，在 Core 层内部转回实际类型。

### 4.5 软件总线 (bit-bang I2C/SPI) — 双分层架构

项目使用软件模拟的 I2C 和 SPI（非硬件外设），原因：
- 灵活性高，不受硬件 I2C/SPI 实例限制
- 跨平台一致性好（软件模拟行为相同）
- 引脚映射自由

**架构：**

```
API/API_I2C/API_I2C.c          ← 协议逻辑 (平台无关, 始终编译)
    │  Start/Stop/SendByte/ReceiveByte/Wait_Ack/...
    │  通过 soft_i2c_hal.h 桥接 ↓

Core/STM32F407/f407_soft_i2c/  ← GPIO 翻转+延时 (CMake 按平台选一个)
    │  直接寄存器访问: BSRR/BRR
```

```
API/API_SPI/API_SPI.c          ← 协议逻辑 (Start/Stop/SwapByte)
    │  通过 soft_spi_hal.h 桥接 ↓
Core/STM32F407/f407_soft_spi/  ← GPIO 翻转+延时
```

**HAL 桥接接口** (`soft_i2c_hal.h`, `soft_spi_hal.h`)：
- API 协议层 ↔ Core 底层实现之间的桥梁
- **不对外暴露**：BSP/App 层不应直接引用，统一通过 `API_I2C.h` / `API_SPI.h` 操作
- 声明平台无关的底层原语函数（W_SCL, W_SDA, R_SDA, delay_us 等）
- 由 Core 层各平台各自实现（零 `#if`，直接寄存器操作）

**设计原则**：
- 所有 BSP 设备只通过 `API_I2C.h` / `API_SPI.h` 的标准协议函数操作总线
- 总线选择+速率集中配置在 [SYSTEM/BusRate.h](SYSTEM/BusRate.h)

### 4.6 中断优先级统一管理 (IrqPriority.h)

`SYSTEM/IrqPriority.h` 集中管理所有 NVIC 中断优先级，是飞控实时性的保障：

```
IrqPriority.h (策略层)  →  API/Core (机制层)  →  NVIC 硬件寄存器
  "谁比谁高"                   "怎么设"              "硬件执行"
```

**优先级分配（飞控当前策略）**：

| 优先级 | 中断源 | 理由 |
|:---:|--------|------|
| 0 | SysTick | 系统心跳基准 |
| 1 | TIM3 (API_TIM1) | 1ms 控制节拍，所有 PID 回路的心脏 |
| 2 | MPU6050 EXTI | DMP 姿态数据就绪，串级控制外环输入，实时性高于速度环 |
| 3 | TIM2 (API_TIM2) | 串口打印 / 时间戳 |
| 4 | USART ×2 | 通信（丢包可重传，优先级最低） |

**调用链（以 MPU6050 为例）**：
```
IrqPriority.h: #define IRQ_PRIO_MPU6050 2U
    → Enroll.c: Enroll_MPU6050_Register() → API_EXTI_Init(id, trigger, IRQ_PRIO_MPU6050, ...)
    → API/exti.c: API_EXTI_CoreInit(port, pin, ..., preemptPriority=2, ...)
    → Core/f407_exti.c: NVIC_SetPriority(irqn, 2) — 写入硬件寄存器
```

**多 MCU 适配**：
- STM32F407：Cortex-M4，4bit NVIC → 0~15 级，每级独立
- STM32F103：Cortex-M3，4bit NVIC → 0~15 级
- MSPM0G3507：Cortex-M0+，2bit NVIC → 0~3 级，需压缩

---

## 5. 飞控控制架构

### 5.1 串级 PID 结构

```
目标角度 (遥控器/预设)
     │
     ▼
┌─────────────────┐
│  角度环 PID      │  外环 (Pitch / Roll / Yaw)
│  pid_pitch       │  输入: 目标角 - DMP 姿态角
│  pid_roll        │  输出: 目标角速度
│  pid_yaw         │  频率: 250Hz
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  角速度环 PID    │  内环
│  pid_rate_pitch  │  输入: 目标角速度 - MPU6050 陀螺
│  pid_rate_roll   │  输出: 电机 PWM 修正量
│  pid_rate_yaw    │  频率: 500Hz
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  混控矩阵        │  X 型四轴
│  Motor_Mix()     │  油门 + 姿态修正 → 4路PWM
└────────┬────────┘
         │
         ▼
  4 路 ESC 信号 (TIM1 CH1~CH4, 50Hz)
```

### 5.2 中断回调与任务标志位

飞控使用"中断置标志 → 主循环轮询执行"模式：

```c
// 定时器中断 1ms 一次
void Control_Task1_Callback(void) {
    static uint8_t tick = 0;
    tick++;
    if (tick % 1 == 0) pid_task_flag = 1;    // 500Hz 姿态环 (2ms * 1)
    if (tick % 1 == 0) speed_task_flag = 1;   // 500Hz 速度环
}

// 主循环
while (1) {
    mpu_angle();  // 读取 DMP 姿态
    if (pid_task_flag) {
        pid_task_flag = 0;
        PID_Pitch_Roll_Combined(Pitch, Roll);  // 串级控制
    }
}
```

### 5.3 X 型混控矩阵

```
       M1(+)           M2(-)
              ↖  ↗
               🤖
              ↙  ↗
       M3(-)           M4(+)

M1 = Throttle + Pitch - Roll - Yaw   (右前, CW)
M2 = Throttle - Pitch - Roll + Yaw   (左前, CCW)
M3 = Throttle + Pitch + Roll + Yaw   (左后, CW)
M4 = Throttle - Pitch + Roll - Yaw   (右后, CCW)
```

输出经 `MOTOR_MIX_LIMIT (2047.0f)` 限幅后写入 PWM 占空比。

---

## 6. F407 传感器与引脚配置

### 6.1 传感器总线拓扑

```
                    PB8 (SCL)  ← 软件 I2C
                    PB9 (SDA)
                       │
          ┌────────────┼────────────┐
          │            │            │
       MPU6050     QMC5883P     BMP280
      (0x68)       (0x0D)       (0x76)
          │
       PE7 (INT)  →  EXTI 中断
```

### 6.2 完整引脚分配表

| 功能 | 引脚 | 外设资源 | 说明 |
|------|------|----------|------|
| I2C SCL | PB8 | 软件 I2C (API_I2C1) | 三传感器共用 |
| I2C SDA | PB9 | 软件 I2C (API_I2C1) | 三传感器共用 |
| MPU6050 INT | PE7 | EXTI | DMP 数据就绪 |
| SPI SCK | PA5 | 软件 SPI (API_SPI1) | NRF24L01 |
| SPI MOSI | PA7 | 软件 SPI (API_SPI1) | NRF24L01 |
| SPI MISO | PA6 | 软件 SPI (API_SPI1) | NRF24L01 |
| SPI CS | PC4 | GPIO | NRF24L01 片选 |
| NRF24L01 CE | PC5 | GPIO | 收发模式控制 |
| 电机1 | PE9 | TIM1 CH1 | ESC1 |
| 电机2 | PE11 | TIM1 CH2 | ESC2 |
| 电机3 | PE13 | TIM1 CH3 | ESC3 |
| 电机4 | PE14 | TIM1 CH4 | ESC4 |
| USART1 TX | PA9 | USART1 | 调试 (115200) |
| USART1 RX | PA10 | USART1 | 调试 |
| USART3 TX | PD8 | USART3 | 无线串口 |
| USART3 RX | PD9 | USART3 | 无线串口 |
| LED1 | PE2 | GPIO | 绿色 |
| LED2 | PE3 | GPIO | 红色 |
| LED3 | PE4 | GPIO | 蓝色 |

---

## 7. 当前 API 层支持的外设接口

| API 头文件 | 功能 | F103 | F407 | G3507 |
|-----------|------|:---:|:---:|:---:|
| [gpio.h](API/inc/gpio.h) | GPIO 输入/输出 | ✅ | ✅ | ✅ |
| [usart.h](API/inc/usart.h) | 串口通信 | ✅ | ✅ | ✅ |
| [pwm.h](API/inc/pwm.h) | PWM 输出 | ✅ | ✅ | ✅ |
| [tim.h](API/inc/tim.h) | 定时器中断 | ✅ | ✅ | ✅ |
| [adc.h](API/inc/adc.h) | ADC 采集 | ✅ | ✅ | ✅ |
| [exti.h](API/inc/exti.h) | 外部中断 | ✅ | ✅ | ✅ |
| [Encoder.h](API/inc/Encoder.h) | 编码器接口 | ✅ (TIM) | ✅ (TIM) | ✅ (EXTI) |
| [API_I2C.h](API/API_I2C/API_I2C.h) | 软件 I2C 协议 | ✅ | ✅ | ✅ |
| [API_SPI.h](API/API_SPI/API_SPI.h) | 软件 SPI 协议 | ✅ | ✅ | ✅ |

---

## 8. BSP 层当前支持的器件

| 模块 | 文件 | 接口类型 | F103 | F407 | G3507 |
|------|------|---------|:---:|:---:|:---:|
| LED | [LED.c](BSP/LED/LED.c) | GPIO 输出 | ✅ | ✅ | ✅ |
| KEY | [KEY.c](BSP/KEY/KEY.c) | GPIO 输入 (消抖) | ✅ | ✅ | ✅ |
| OLED | [OLED.c](BSP/OLED/OLED.c) | SPI / I2C 双模式 | ✅ | ✅ | ✅ |
| MPU6050 | [MPU6050.c](BSP/MPU6050/MPU6050.c) | I2C + EXTI | ✅ | ✅ | ✅ |
| MPU6050 DMP | [eMPL/](BSP/MPU6050/eMPL/) | InvenSense 官方 DMP | ✅ | ✅ | ✅ |
| QMC5883P | [QMC5883P.c](BSP/QMC5883P/QMC5883P.c) | I2C | — | ✅ | — |
| BMP280 | [BMP280.c](BSP/BMP280/BMP280.c) | I2C | — | ✅ | — |
| NRF24L01 | [NRF24L01.c](BSP/NRF24L01/NRF24L01.c) | SPI | ✅ | ✅ | — |
| TB6612 | [TB6612.c](BSP/TB6612/TB6612.c) | PWM + GPIO | ✅ | ✅ | ✅ |

> 注：QMC5883P、BMP280 当前仅 F407 实现；NRF24L01 当前仅 F103/F407 实现。

---

## 9. SYSTEM 层与 Core exti/sys 文件角色说明

### 为什么 103/407 只有 `exti.h` 和 `sys.c`？

STM32 的 `SystemInit()` 在启动文件中由 CMSIS 标准库自动调用完成时钟配置，所以 Core 层不需要额外的时钟初始化代码。

| MCU | 文件 | 实际作用 |
|-----|------|---------|
| **F407** | `src/f407_exti.c` | EXTI 初始化（SYSCFG + EXTI + NVIC） |
| | `inc/f407_exti.h` | EXTI 函数声明 |
| | 无时钟 sys | 不需要 — `SystemInit()` 已配好 168MHz |
| **F103** | `src/f103_exti.c` | EXTI 初始化（AFIO + EXTI + NVIC） |
| | `inc/f103_exti.h` | EXTI 函数声明 |
| **G3507** | `G3507_sys.c/h` | 时钟初始化（80MHz PLL）+ 系统信息查询 |

### SYSTEM 层 vs Core 层分工

```
SYSTEM/sys.c                          ← 平台无关的门面层
  ├─ SYS_Init()                       → 条件编译分发
  │   └─ F103/407: 空（SystemInit 已做）
  │   └─ G3507: G3507_SYS_Init()      → 配 80MHz PLL
  ├─ SYS_EXTI_GetIrqn(port, pin)      → 端口+引脚 → NVIC IRQ 号
  └─ SYS_EXTI_GetLineIndex(pin)       → 引脚掩码 → 0~15 线号

Core/STM32F407/src/f407_exti.c         ← EXTI 硬寄存器实现 (SYSCFG 体系)
Core/STM32F103/src/f103_exti.c         ← EXTI 硬寄存器实现 (AFIO 体系)
Core/MSPM0G3507/G3507_sys.c           ← 系统时钟初始化 (80MHz PLL)
```

---

## 10. 关键文件索引

### 构建系统
| 文件 | 作用 |
|------|------|
| [CMakeLists.txt](CMakeLists.txt) | 统一构建入口，含平台分支逻辑 |
| [CMakePresets.json](CMakePresets.json) | CMake 预设 (Debug/F103/F407/G3507) |
| [gcc-arm-none-eabi.cmake](gcc-arm-none-eabi.cmake) | ARM GCC 交叉编译工具链 |
| [.vscode/tasks.json](.vscode/tasks.json) | VS Code 构建/烧录任务 |
| [.vscode/settings.json](.vscode/settings.json) | VS Code CMake 源目录配置 |

### 注册层 (最需要关注的目录)
| 文件 | 作用 |
|------|------|
| [Enroll/Enroll.h](Enroll/Enroll.h) | 注册层对外接口 + `ENROLL_MCU_TARGET` 默认定义 |
| [Enroll/Enroll.c](Enroll/Enroll.c) | X-Macro 展开 + Register 门面函数 |
| [Enroll/Enroll_Internal.h](Enroll/Enroll_Internal.h) | 仅 Enroll.c 使用的内部依赖 |
| [Enroll/407_hw_config.h](Enroll/407_hw_config.h) | **★当前主力**：F407 板级引脚映射 |
| [Enroll/103_hw_config.h](Enroll/103_hw_config.h) | F103 板级引脚映射 |
| [Enroll/G3507_hw_config.h](Enroll/G3507_hw_config.h) | G3507 板级引脚映射 |

### SYSTEM 层
| 文件 | 作用 |
|------|------|
| [SYSTEM/sys.c](SYSTEM/sys.c) | SYS_Init / EXTI_GetIrqn / LineIndex 门面 |
| [SYSTEM/sys.h](SYSTEM/sys.h) | 系统初始化和 EXTI 辅助接口声明 |
| [SYSTEM/Delay.h](SYSTEM/Delay.h) | 统一延时接口（Delay_us/ms/s） |
| [SYSTEM/BusRate.h](SYSTEM/BusRate.h) | 软件总线选择+速率集中配置 |
| [SYSTEM/IrqPriority.h](SYSTEM/IrqPriority.h) | **★飞控关键**：NVIC 中断优先级统一管理 |

### 飞控应用层
| 文件 | 作用 |
|------|------|
| [A_Entry/main.c](A_Entry/main.c) | 程序入口，完整初始化流程 + 飞控主循环 |
| [app/Control/Control.c](app/Control/Control.c) | 串级 PID 控制 + 混控 + 电机加载 |
| [app/Control_Task/](app/Control_Task/) | 中断回调 + 任务标志位管理 |
| [app/PID/](app/PID/) | PID 控制器实现 |
| [app/Filter/](app/Filter/) | 滤波器实现 |
| [app/My_Usart/](app/My_Usart/) | 串口打印封装 |

### 板级驱动（飞控核心传感器）
| 文件 | 作用 |
|------|------|
| [BSP/MPU6050/MPU6050.c](BSP/MPU6050/MPU6050.c) | MPU6050 初始化 + 寄存器读写 |
| [BSP/MPU6050/MPU6050_Int.c](BSP/MPU6050/MPU6050_Int.c) | DMP 数据读取 + 姿态角计算 |
| [BSP/MPU6050/eMPL/](BSP/MPU6050/eMPL/) | InvenSense 官方 DMP 库 |
| [BSP/QMC5883P/](BSP/QMC5883P/) | QMC5883P 磁力计驱动 |
| [BSP/BMP280/](BSP/BMP280/) | BMP280 气压计驱动 |
| [BSP/NRF24L01/](BSP/NRF24L01/) | NRF24L01 2.4G 无线驱动 |

### 总线层
| 文件 | 作用 |
|------|------|
| [API/API_I2C/](API/API_I2C/) | 软件 I2C 协议层 (平台无关) |
| [API/API_SPI/](API/API_SPI/) | 软件 SPI 协议层 (平台无关) |
| [Core/STM32F407/f407_soft_i2c/](Core/STM32F407/f407_soft_i2c/) | F407 I2C GPIO 翻转+延时 |
| [Core/STM32F407/f407_soft_spi/](Core/STM32F407/f407_soft_spi/) | F407 SPI GPIO 翻转+延时 |

---

## 11. 开发约定与命名规范

### 函数命名
- `API_xxx_*` — API 层对外接口 (如 `API_GPIO_Write`)
- `F407_xxx_*` — F407 Core 层实现
- `F103_xxx_*` — F103 Core 层实现
- `G3507_xxx_*` — G3507 Core 层实现
- `Enroll_xxx_*` — 注册层门面函数
- `API_I2C_*` / `API_SPI_*` — 软件总线协议层
- `soft_i2c_hal_*` / `soft_spi_hal_*` — 总线 HAL 桥接接口 (由 Core 层实现)

### 文件组织
- 每个外设模块在自己的目录下，含 `.c` + `.h`
- API 层: `inc/` 放头文件，`src/` 放实现
- Core 层: `inc/` 放头文件，`src/` 放实现，`cmake/` 放链接脚本
- BSP 模块: 头文件与源文件平级放在模块目录

### 条件编译
- 统一通过 `ENROLL_MCU_TARGET` 宏分发，不引入额外的 feature flag
- `ENROLL_MCU_TARGET` 默认值定义在 [Enroll/Enroll.h](Enroll/Enroll.h) 中，当前为 `ENROLL_MCU_F407`

### 平台差异处理
- 通用代码放各层通用列表
- 平台差异代码在各 MCU 分支中独立维护
- BSP 器件按"通用列表 + 平台追加"组织

---

## 12. 飞控初始化流程

`main.c` 中的初始化顺序有严格依赖关系，不可随意调换：

```
1. SYS_Init()                    ← 系统时钟（168MHz）
2. Enroll_xxx_Register() × 6     ← 登记所有资源配置表
3. API_TIM_RegisterIrqHandler()  ← 绑定中断回调（必须在 API_TIM_Init 之前）
4. API_USART_Init()              ← 调试串口
5. API_PWM_Init()                ← PWM 输出 (50Hz ESC)
6. API_TIM_Init()                ← 控制节拍 (1ms)
7. API_I2C_Init()                ← 软件 I2C 总线
8. API_SPI_Init()                ← 软件 SPI 总线
9. LED_Init()                    ← 状态指示灯
10. MPU_Init()                   ← MPU6050 初始化
11. mpu_dmp_init()               ← DMP 初始化
12. Enroll_MPU6050_Register()    ← MPU6050 INT 中断（DMP 初始化后才能使能）
13. QMC_Init()                   ← 磁力计初始化
14. BMP280Init()                 ← 气压计初始化
15. 进入主循环
```

---

## 13. 新增 MCU 的接入步骤

1. **Drivers 层** — 在 `Drivers/` 下新增 `Drivers_NewMCU/` 目录，放入启动文件 + SDK/HAL
2. **Core 层** — 在 `Core/NewMCU/` 下实现 GPIO/USART/PWM/TIM/EXTI/I2C/SPI 等底层驱动
3. **Enroll 层** — 在 `Enroll/` 下新增 `NewMCU_hw_config.h`，定义所有板级映射宏
4. **CMakeLists.txt** — 新增 `elseif(RESOLVED_MCU_TARGET STREQUAL "NewMCU")` 分支
5. **CMakePresets.json** — 新增 `Debug-NewMCU` 构建预设
6. **OpenOCD** — 新增对应的 `.cfg` 下载配置
7. **SYSTEM/BusRate.h** — 新增该 MCU 的总线选择+速率配置

---

## 14. 当前工程状态

### 已完成
- OmniLayer 八层架构完整迁移至 OmniFlight
- F407 板级引脚映射 (`407_hw_config.h`)
- 软件 I2C/SPI 双分层架构 (API 协议 + Core GPIO 翻转)
- 所有 BSP 设备统一通过 API_I2C/API_SPI 操作总线
- 总线选择+速率集中至 `SYSTEM/BusRate.h`
- 中断优先级集中至 `SYSTEM/IrqPriority.h`
- MPU6050 DMP 姿态解调完整集成
- 串级 PID 架构 (外环角度 + 内环角速度)
- X 型四轴混控矩阵
- 控制任务中断回调 + 标志位机制
- 串口调试打印

### 待完成
- 油门协议解析（遥控器 → 目标油门/Pitch/Roll/Yaw）
- 电机混控加载与 ESC 输出
- QMC5883P 地磁数据融合（航向角修正）
- BMP280 气压定高
- NRF24L01 无线通信协议对接

---

## 15. 快速上手检查清单

为新对话恢复认知时，按以下顺序阅读关键文件：

1. ✅ 本文档 ([docs/arch-guide.md](docs/arch-guide.md)) — 架构全貌
2. [README.md](README.md) — 项目简介、硬件配置、构建命令
3. [A_Entry/main.c](A_Entry/main.c) — 飞控初始化流程与主循环
4. [Enroll/Enroll.h](Enroll/Enroll.h) — 注册层接口全览
5. [Enroll/407_hw_config.h](Enroll/407_hw_config.h) — **★当前主力 MCU 的板级映射**
6. [app/Control/Control.h](app/Control/Control.h) — 飞控核心接口
7. [SYSTEM/IrqPriority.h](SYSTEM/IrqPriority.h) — 中断优先级策略
8. [SYSTEM/BusRate.h](SYSTEM/BusRate.h) — 软件总线速率策略
9. [API/API_I2C/API_I2C.h](API/API_I2C/API_I2C.h) — I2C 协议层接口
10. [API/API_SPI/API_SPI.h](API/API_SPI/API_SPI.h) — SPI 协议层接口
