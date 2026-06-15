# OmniFlight — 四轴飞控 ✈️

基于 [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) 分层架构框架构建的 STM32F407 四轴飞行控制器。

## 🚀 项目定位

OmniFlight = **OmniLayer 架构 × 飞控算法**，在一块 F407 上跑完整的四轴飞控。

- 同一套飞控代码，切换 Enroll 层硬件映射可跨 F407 / F103 / G3507 编译
- 复用 OmniLayer 的 CMake + GCC + OpenOCD 全工具链
- 积累的 PID、滤波器、传感器驱动、混控算法可复用于后续机器人项目

## ✨ 亮点

- 🧭 **多目标飞控** — 同一套代码，F407 / F103 / G3507 切换编译
- 🧱 **八层架构** — A_Entry / app / BSP / Enroll / API / Core / SYSTEM / Drivers 职责分明
- 🎛️ **串级 PID** — 外环角度 + 内环角速度，500Hz 控制节拍
- 📡 **传感器融合** — MPU6050 六轴 DMP + QMC5883P 地磁 + BMP280 气压
- 🚌 **DShot300** — 数字油门协议，DMA burst 驱动 4 路无刷电调
- 🛰️ **2.4G 遥控** — NRF24L01 无线收发，双向遥测回传
- ⚙️ **注册层（Enroll）** — X-Macro 编译期映射，换 MCU 只改一张配置表
- 🚌 **软件总线** — I2C/SPI 协议与底层分离，速率集中配置

## ⚙️ 硬件配置

| 项目 | 型号 | 接口 |
|------|------|------|
| 主控 | STM32F407VET6 | Cortex-M4 + FPU, 168MHz |
| 陀螺仪 | MPU6050 | I2C + EXTI |
| 磁力计 | QMC5883P | I2C |
| 气压计 | BMP280 | I2C |
| 无线 | NRF24L01 | SPI |
| 电调 | BLHeli_S / BLHeli_32 | DShot300 |
| 蜂鸣器 | 无源 | TIM3 CH4 PWM |

### 引脚分配

| 功能 | 引脚 | 外设 |
|------|------|------|
| I2C SCL / SDA | PB8 / PB9 | 软件 I2C |
| MPU6050 INT | PE7 | EXTI |
| NRF24L01 (SCK/MOSI/MISO/CS/CE) | PA5/PA7/PA6/PC4/PC5 | SPI1 |
| 电机 1~4 | PE9/PE11/PE13/PE14 | TIM1 CH1~4 |
| 蜂鸣器 | PB1 | TIM3 CH4 |
| 调试串口 | PA9/PA10 | USART1 |
| 无线串口 | PD8/PD9 | USART3 |
| LED 1~3 | PE2/PE3/PE4 | GPIO |

## 📁 项目结构

```text
OmniFlight/
├─ A_Entry/main.c              # 飞控主循环
├─ app/
│  ├─ Control/                 # 串级 PID + 混控
│  ├─ Control_Task/            # 中断回调 + 任务标志位
│  ├─ PID/                     # PID 控制器
│  ├─ Filter/                  # 滤波器
│  └─ My_Usart/                # 串口管理 + printf
├─ BSP/
│  ├─ MPU6050/                 # 六轴陀螺仪 + DMP
│  ├─ QMC5883P/                # 磁力计
│  ├─ BMP280/                  # 气压计
│  ├─ NRF24L01/                # 2.4G 无线模块
│  ├─ Dshot/                   # DShot300 油门协议
│  ├─ Motor/                   # 电机混控
│  ├─ Buzzer/                  # 蜂鸣器
│  ├─ LED/ KEY/ OLED/          # 基础外设
│  └─ TB6612/                  # 直流电机驱动（预留）
├─ API/                        # 片内外设抽象层
│  ├─ inc/ src/                # gpio/adc/pwm/tim/usart/exti
│  ├─ API_I2C/                 # 软件 I2C 协议层
│  └─ API_SPI/                 # 软件 SPI 协议层
├─ Enroll/                     # ★ 硬件资源注册中心
├─ Core/                       # 芯片底层实现（F103/F407/G3507）
├─ Drivers/                    # 启动文件 + CMSIS
├─ SYSTEM/                     # sys/Delay/BusRate/IrqPriority
├─ OpenOCD/                    # 下载配置
└─ docs/arch-guide.md          # 架构深度解析
```

## 🏗️ 飞控核心

```
遥控器 (NRF24L01)
     │ speed_temp + Target
     ▼
┌──────────┐    ┌──────────┐    ┌──────────┐
│ 角度环 PID │ → │ 角速度环 PID│ → │ 混控矩阵  │ → DShot_Write()
└──────────┘    └──────────┘    └──────────┘
     外环 250Hz      内环 500Hz       X 型四轴
```

## 🎯 中断优先级

| 优先级 | 中断源 | 理由 |
|:---:|------|------|
| 0 | SysTick | 系统心跳 |
| 1 | TIM3 (1ms) | PID 控制节拍 |
| 2 | MPU6050 EXTI | 姿态数据 |
| 3 | TIM2 | printf / 时间戳 |
| 4 | USART1/3 | 通信 |

## ⚙️ 构建与烧录

| 快捷键 | 功能 |
|--------|------|
| `F7` | 编译（Debug 预设） |
| `F8` | 烧录 |
| `Ctrl+Shift+F1` | 选择 MCU 后编译 |
| `Ctrl+Shift+F2` | 选择 MCU 后下载 |
| `Ctrl+Shift+F3` | 切换默认 MCU 目标 |

```bash
cmake --preset Debug-F407
cmake --build --preset Debug-F407
```

## 📖 详细文档

- 工程架构深度解析：[docs/arch-guide.md](docs/arch-guide.md)
- 架构框架：[OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git)

## ⚠️ 注意事项

- 主力维护 VS Code + CMake 环境，F407 为目标 MCU
- F103 / G3507 架构保留，暂不开发
- DShot 电调需从最低油门（48）逐步递增，不可直接跳到大油门值

## 📮 联系

- QQ 邮箱：634591772@qq.com
