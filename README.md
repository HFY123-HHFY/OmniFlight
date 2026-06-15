# OmniFlight — 四轴飞控

基于 [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) 分层架构框架构建的四轴飞行控制器，
以 STM32F407VET6 为主控，集成 MPU6050 陀螺仪、QMC5883P 磁力计、BMP280 气压计与 NRF24L01 2.4G 收发器。

## 🚀 项目定位

OmniFlight 的核心目标是**在 OmniLayer 分层架构之上，构建一个可稳定飞行的四轴飞控**。

- 同一套飞控代码，通过切换 Enroll 层硬件映射，可跨 F407 / F103 / G3507 多款 MCU 编译运行。
- 复用 OmniLayer 的 CMake + GCC + OpenOCD 全工具链，无需重新搭建开发环境。
- 积累的 PID、滤波器、传感器驱动、混控算法等模块，可复用于后续循迹小车、平衡车等移动机器人项目。

## 🌿 分支策略

| 分支 | 定位 | 状态 |
|------|------|------|
| `main` | 裸机飞控主线（当前开发） | 持续维护 |

## ✨ 项目亮点

- 🧭 **多目标飞控**：同一套飞控代码，支持 F407 / F103 / G3507 多 MCU 切换编译。
- 🧱 **分层架构**：继承 OmniLayer 八层架构，应用逻辑与芯片实现解耦，可迁移、可扩展。
- 🎛️ **串级 PID**：外环角度环 + 内环角速度环，控制响应快、超调低。
- 📡 **传感器融合**：MPU6050 六轴 DMP + QMC5883P 地磁 + BMP280 气压，姿态估算稳定可靠。
- 🚌 **混控矩阵**：X 型四轴混控，电机输出限幅保护。
- ⚙️ **参数调优**：支持串口/遥控器实时调参，飞行中修改 PID 参数。
- 🛰️ **2.4G 遥控**：NRF24L01 无线收发，配合遥控器实现双向通信。

## ⚙️ 硬件配置

| 项目 | 型号 | 接口 | 备注 |
|------|------|------|------|
| 主控 | STM32F407VET6 | — | Cortex-M4 + FPU, 168MHz, 512KB Flash, 192KB RAM |
| 陀螺仪 | MPU6050 | I2C + EXTI | 六轴惯性测量，内置 DMP 姿态解算 |
| 磁力计 | QMC5883P | I2C | 三轴地磁，辅助航向角解算 |
| 气压计 | BMP280 | I2C | 气压/温度，辅助定高 |
| 无线收发 | NRF24L01 | SPI | 2.4GHz，与遥控器双向通信 |
| 电调 | BLHeli / SimonK | PWM (50Hz) | 4 路 TIM1 输出，标准 ESC 协议 |
| 电机 | 1104 无刷直流电机 | — | 经由电调驱动 |

### 引脚分配（F407 飞控板）

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| I2C SCL | PB8 | 软件 I2C | MPU6050 / QMC5883P / BMP280 共用 |
| I2C SDA | PB9 | 软件 I2C | 同上 |
| MPU6050 INT | PE7 | EXTI | DMP 数据就绪中断 |
| NRF24L01 SCK | PA5 | 软件 SPI | SPI 时钟 |
| NRF24L01 MOSI | PA7 | 软件 SPI | 主机输出 |
| NRF24L01 MISO | PA6 | 软件 SPI | 从机输入 |
| NRF24L01 CS | PC4 | GPIO | SPI 片选 |
| NRF24L01 CE | PC5 | GPIO | 收发模式控制 |
| 电机1 | PE9 | TIM1 CH1 | ESC 信号 |
| 电机2 | PE11 | TIM1 CH2 | ESC 信号 |
| 电机3 | PE13 | TIM1 CH3 | ESC 信号 |
| 电机4 | PE14 | TIM1 CH4 | ESC 信号 |
| USART1 TX | PA9 | USART1 | 调试串口 (115200bps) |
| USART1 RX | PA10 | USART1 | 调试串口 |
| USART3 TX | PD8 | USART3 | 无线串口 |
| USART3 RX | PD9 | USART3 | 无线串口 |
| LED1 | PE2 | GPIO | 绿色 |
| LED2 | PE3 | GPIO | 红色 |
| LED3 | PE4 | GPIO | 蓝色 |

### 传感器总线拓扑

```
                    PB8 (SCL)
                    PB9 (SDA)
                       │
          ┌────────────┼────────────┐
          │            │            │
       MPU6050     QMC5883P     BMP280
       (0x68)      (0x0D)       (0x76)
          │
       PE7 (INT)
```

## 📁 项目结构

```text
OmniFlight/
├─ A_Entry/                    # 程序入口 (main.c)
│  └─ main.c                   #   飞控主循环：注册 → 初始化 → 控制
├─ API/                        # MCU 片内外设抽象接口层
│  ├─ inc/                     #   gpio / adc / pwm / tim / usart / exti 等
│  ├─ src/                     #   API 实现（条件编译分发到 Core）
│  ├─ API_I2C/                 #   软件 I2C 协议层（平台无关）
│  └─ API_SPI/                 #   软件 SPI 协议层（平台无关）
├─ app/                        # 应用层：飞控算法与业务逻辑
│  ├─ Control/                 #   串级 PID 控制、混控、电机加载
│  ├─ Control_Task/            #   控制任务调度与中断回调
│  ├─ PID/                     #   PID 控制器（位置式/增量式）
│  ├─ Filter/                  #   滤波器（低通/互补/卡尔曼）
│  └─ My_Usart/                #   串口数据管理与格式化打印
├─ BSP/                        # 板级支持层：板载器件驱动封装
│  ├─ LED/                     #   LED 控制
│  ├─ KEY/                     #   按键检测（消抖）
│  ├─ OLED/                    #   OLED 显示屏 (I2C/SPI 双模式)
│  ├─ MPU6050/                 #   MPU6050 六轴传感器 + DMP 驱动
│  ├─ QMC5883P/                #   QMC5883P 三轴磁力计
│  ├─ BMP280/                  #   BMP280 气压传感器
│  ├─ NRF24L01/                #   NRF24L01 2.4G 无线模块
│  └─ TB6612/                  #   TB6612 电机驱动（预留）
├─ Core/                       # 芯片底层实现（按 MCU 分目录）
│  ├─ STM32F103/               #   F103: soft_i2c / soft_spi / src / inc / cmake
│  ├─ STM32F407/               #   F407: soft_i2c / soft_spi / src / inc / cmake
│  └─ MSPM0G3507/              #   G3507: soft_i2c / soft_spi / src / inc / cmake
├─ Drivers/                    # 启动文件、CMSIS / 标准外设库
│  ├─ Drivers_STM32F1/         #   STM32F1 标准外设库 + 启动文件
│  ├─ Drivers_STM32F4/         #   STM32F4 标准外设库 + 启动文件
│  └─ Drivers_M0G3507/         #   TI DriverLib + CMSIS
├─ Enroll/                     # 硬件资源注册与板级映射（★核心特色）
│  ├─ Enroll.h                 #   注册层对外接口 + 默认 MCU 目标
│  ├─ Enroll.c                 #   X-Macro 展开 + Register 门面函数
│  ├─ Enroll_Internal.h        #   内部依赖（仅 Enroll.c 使用）
│  ├─ 103_hw_config.h          #   F103 板级引脚映射
│  ├─ 407_hw_config.h          #   F407 板级引脚映射
│  └─ G3507_hw_config.h        #   G3507 板级引脚映射
├─ SYSTEM/                     # 系统级配置与初始化
│  ├─ sys.c / sys.h            #   系统初始化 / 中断分发
│  ├─ Delay.h                  #   统一延时接口
│  ├─ BusRate.h                #   软件总线速率 + 总线选择集中配置
│  └─ IrqPriority.h            #   NVIC 中断优先级统一管理
├─ OpenOCD/                    # 下载配置（F103 / F407 / G3507）
├─ MDK_ARM/                    # Keil 工程（保留兼容）
├─ build/                      # 构建输出目录
├─ docs/                       # 项目文档
│  └─ arch-guide.md            #   工程架构深度解析
├─ CMakeLists.txt              # 统一构建入口
├─ CMakePresets.json           # 构建预设（Debug / F103 / F407 / G3507）
└─ gcc-arm-none-eabi.cmake     # GCC ARM 交叉编译工具链
```

## 🏗️ 分层说明

| 层级 | 目录 | 职责 |
|------|------|------|
| 入口层 | `A_Entry/` | 唯一 `main.c`，飞控主循环：系统初始化 → 注册 → 外设 → 传感器 → 控制 |
| 应用层 | `app/` | 飞控核心：串级 PID、混控、滤波器、控制任务调度 |
| 接口层 | `API/` | 统一 GPIO/USART/TIM/PWM/I2C/SPI 片内外设接口，屏蔽芯片差异 |
| 板级层 | `BSP/` | 封装板载器件（MPU6050/QMC/BMP/NRF/LED…），向上提供稳定设备接口 |
| 注册层 | `Enroll/` | ★硬件资源注册中心：板级引脚映射 → 逻辑外设 ID，切换 MCU 只改此层 |
| 核心层 | `Core/` | 寄存器级实现：GPIO 翻转、TIM 配置、NVIC 操作，按 MCU 分目录 |
| 系统层 | `SYSTEM/` | 系统初始化、延时、总线速率、中断优先级统一配置 |
| 驱动层 | `Drivers/` | 启动文件、CMSIS、标准外设库、TI DriverLib |

## 🧩 注册层（Enroll）特色

`Enroll/` 是本项目最有辨识度的一层，本质是**"硬件资源注册中心"**：

- 用编译期 X-Macro 将"逻辑外设 ID"映射到"物理引脚 + 硬件实例"。
- 让上层 BSP / APP 不需要关心不同芯片的引脚差异。
- 切换 MCU 时，只需提供新的 `xxx_hw_config.h`，业务代码零改动。

```
407_hw_config.h (板级映射宏)
     ↓ 定义 HW_PWM_MAP / HW_I2C_MAP / HW_USART_MAP 等宏
Enroll.c (X-Macro 展开)
     ↓ 展开为结构体配置表
Enroll_xxx_Register() (门面函数)
     ↓ 传入配置表
API / BSP 的 Register / Init 函数
```

## 🎛️ 飞控核心算法

### 串级 PID 控制架构

```
目标角度 (遥控器)
     │
     ▼
┌─────────────┐
│  角度环 PID  │  ← 外环：Pitch / Roll / Yaw 角度控制
└──────┬──────┘
       │ 输出 = 目标角速度
       ▼
┌─────────────┐
│ 角速度环 PID │  ← 内环：角速度控制 (MPU6050 陀螺数据)
└──────┬──────┘
       │ 输出 = 电机 PWM 修正量
       ▼
┌─────────────┐
│  混控矩阵    │  ← X 型四轴混控
└──────┬──────┘
       │
       ▼
  4 路 PWM → ESC → 电机
```

### 控制节拍

| 环 | 频率 | 定时器 | 说明 |
|:---|:---:|------|------|
| 角速度环（内环） | 500Hz | TIM3, 1ms | 陀螺仪数据读取 + 角速度 PID |
| 角度环（外环） | 250Hz | 同上, 2ms 累计 | DMP 姿态角 + 角度 PID |
| 混控输出 | 500Hz | 同上 | 混控矩阵 + PWM 更新 |
| 传感器读取 | 500Hz | MPU6050 INT | DMP 数据就绪中断触发 |

### 混控矩阵（X 型四轴）

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

## 🚌 软件总线（I2C / SPI）

采用 **API 协议 + Core 底层** 双分层架构：

```
API/API_I2C/  (协议逻辑，平台无关)
     │  Start / Stop / SendByte / ReceiveByte / Wait_Ack
     │  通过 soft_i2c_hal.h 桥接
     ▼
Core/STM32F407/f407_soft_i2c/  (GPIO 翻转 + 延时)
     │  直接寄存器访问: BSRR / BRR
```

- 总线速率和总线选择集中在 `SYSTEM/BusRate.h` 配置。
- 所有 BSP 设备只通过 `API_I2C.h` / `API_SPI.h` 操作总线。
- 新增设备 / 调速 / 换总线只需改一个文件。

## 🎯 中断优先级统一管理

飞控对中断实时性要求极高，优先级策略集中在 `SYSTEM/IrqPriority.h` 中：

| 优先级 | 中断源 | 理由 |
|:---:|--------|------|
| 0 | SysTick | 系统心跳基准 |
| 1 | TIM3 (API_TIM1) | 1ms 控制节拍，串级 PID 心脏 |
| 2 | MPU6050 EXTI | DMP 姿态数据就绪，外环输入，实时性高于速度环 |
| 3 | TIM2 (API_TIM2) | 串口打印 / 时间戳 |
| 4 | USART1 / USART3 | 通信（丢包可重传，优先级最低） |

- **原则**：数字越小优先级越高。Core 层只提供"怎么设"的机制，`IrqPriority.h` 决定"谁比谁高"的策略。
- **多 MCU 适配**：F407（Cortex-M4, 4bit NVIC, 0~15）和 F103/G3507 自动按宏展开正确数值。

## ⚙️ 构建与烧录

### ⌨️ VS Code 快捷键

- `F7` — 编译（先配置再构建，对应 Build Debug）
- `F8` — 烧录（先编译再烧录，对应 Flash Debug）
- `Ctrl+Shift+F1` — 弹窗选择 MCU 后编译
- `Ctrl+Shift+F2` — 弹窗选择 MCU 后下载
- `Ctrl+Shift+F3` — 弹窗选择并写入默认 MCU 宏

### 🔧 命令行

```bash
cmake --preset Debug-F407
cmake --build --preset Debug-F407
```

### 🛰️ OpenOCD 烧录

```bash
openocd -f OpenOCD/F407_OpenOCD.cfg
```

## 📌 开发原则

- **业务逻辑不直接操作寄存器**：飞控算法（PID/混控/滤波）只调用 API/BSP 接口。
- **目标相关代码集中在 Core / SYSTEM / Drivers**：切换 MCU 时只换底层。
- **BSP 接口保持稳定**：器件驱动向上暴露统一接口，切换芯片只替换 Enroll 映射与 Core 实现。
- **性能优先**：I2C/SPI 热路径只拆两层（协议 + GPIO 翻转），避免过深封装。
- **中断优先级集中管理**：改优先级只改 `IrqPriority.h` 一行宏。

## 📖 详细文档

- 完整工程架构深度解析：[docs/arch-guide.md](docs/arch-guide.md)
- 该文档定向用于 AI 上下文初始化，新对话中直接加载即可让 AI 快速熟悉项目分层、设计规范与工程逻辑。

## ⚠️ 注意事项

1. 本项目主力维护 VS Code + CMake 编译环境。Keil MDK 配套工程不再同步迭代更新，如需使用 Keil 编译，需手动补齐缺失头文件与工程配置。
2. STM32F103 和 MSPM0G3507 暂不在本项目开发范围内，但其文件架构保留于工程中，不影响 F407 编译。
3. FreeRTOS、USB 协议库源码不再同步上传，如需使用请自行获取。

## 📮 项目状态与联系

- 项目持续开发中。
- 架构框架：[OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git)
- 联系邮箱：634591772@qq.com
