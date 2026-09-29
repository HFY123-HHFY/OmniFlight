# OmniFlight — 四轴飞控 ✈️

基于 [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) 分层架构框架构建的 STM32F407 四轴飞行控制器。
- 🎯 "项目完结 · 已实测稳定定高定点悬停,可手动摇杆控制高度和水平位置（x,y,z）三维姿态" 

## 🚀 项目定位

OmniFlight = **OmniLayer 架构 × 飞控算法**，在一块 F407 上跑完整的四轴飞控。

- 🧩 Enroll 注册层 + X-Macro 编译期映射，让飞控算法与芯片底层解耦（可移植架构）
- 🛠️ 复用 OmniLayer 的 CMake + GCC + OpenOCD 全工具链
- 🧰 积累的 PID、滤波器、传感器驱动、混控算法可复用于后续机器人项目

## ✨ 亮点

- 🧭 **可移植架构** — 算法与芯片解耦，曾移植验证 F103 / MSPM0G3507（见归档 tag）
- 🧱 **八层架构** — A_Entry / app / BSP / Enroll / API / Core / SYSTEM / Drivers 职责分明
- 🖥️ **FreeRTOS 实时内核** — 6 任务 + 信号量 + 互斥锁，抢占式调度接管裸机控制链
- 🎛️ **串级 PID** — 外环角度 + 内环角速度（Pitch/Roll）+ 偏航角速度环（gyro_z 消除自旋）+ 定高环（MTF02P ToF 距离，✅ 已实测定高）+ 定点环（MTF02P 光流位置+速度串级 → 倾角指令，✅ 已实测定点），500Hz 控制节拍
- 📡 **姿态测量** — MPU6050 DMP (Pitch/Roll) + gyro_z 零偏校准偏航角速度环
- 🚌 **DShot300** — 数字油门协议，DMA burst 驱动 4 路无刷电调
- 🛰️ **2.4G 遥控** — NRF24L01 软件 SPI 无线收发，双向遥测回传
- 🛸 **光流测距** — MTF-02P Micolink 协议（USART4），距离 mm + 光流速度 cm/s@1m，定高环 + 定点环数据源（定高/定点均已实测）
- ⚙️ **注册层（Enroll）** — X-Macro 编译期映射，换 MCU 只改一张配置表
- 🚌 **软件总线** — I2C/SPI 协议与底层分离，速率集中配置

## ⚙️ 硬件配置

| 项目 | 型号 | 接口 |
|------|------|------|
| 🧠 主控 | STM32F407VET6 | Cortex-M4 + FPU, 168MHz |
| 🧭 陀螺仪 | MPU6050 | I2C + EXTI |
| 🧲 磁力计 | QMC5883P | I2C（驱动保留，当前未启用） |
| 🌡️ 气压计 | BMP280 | I2C（驱动保留，当前未启用） |
| 📡 无线 | NRF24L01 | 软件 SPI |
| 🛸 光流测距 | MTF-02P | USART4 115200（Micolink） |
| ⚡ 电调 | BLHeli_S / BLHeli_32 | DShot300 |
| 🔔 蜂鸣器 | 无源 | TIM3 CH4 PWM |

### 🔌 引脚分配

| 功能 | 引脚 | 外设 |
|------|------|------|
| I2C SCL / SDA | PB8 / PB9 | 硬件 I2C1 |
| MPU6050 INT | PE7 | EXTI |
| NRF24L01 (SCK/MOSI/MISO/CS/CE) | PA5/PA7/PA6/PC4/PC5 | 软件 SPI |
| 电机 1~4 | PE9/PE11/PE13/PE14 | TIM1 CH1~4 |
| 蜂鸣器 | PB1 | TIM3 CH4 |
| 板载调试串口 | PB6/PB7 | USART1 |
| 光流测距串口 | PA0/PA1 | USART4 |
| 调试串口 | PD5/PD6 | USART2 |
| 无线串口 | PD8/PD9 | USART3 |
| LED 1~3 | PE2/PE3/PE4 | GPIO |

### 🧭 机架方位（X 型四轴，以机头朝上为准）

```text
              机头
     电机1           电机2

左侧                        右侧

     电机4           电机3
              机尾
```

- 🛠️ 电机编号：1 左前 / 2 右前 / 3 右后 / 4 左后
- 🎛️ 混控矩阵（Motor.c，飞行已验证）：pitch 项驱动左/右差速（**横向倾角**）、
  roll 项驱动前/后差速（**纵向倾角**）——故定点环 横向 → Target_Pitch、纵向 → Target_Roll
- 🛸 光流方向（手持实测）：`flow_x` 左→右为正 / `flow_y` 机头→机尾为正
- 🕹️ 定点摇杆（不回中）：`Position_X_Stick_Input` +100 = 向右（flow_x 正）；
  `Position_Y_Stick_Input` +100 = 向前 = 机尾→机头（flow_y 负）；回中 = 悬停定点

## 📁 项目结构

```text
OmniFlight/
├─ A_Entry/main.c              # 飞控初始化 + FreeRTOS 调度器启动
│  FreeRTOSConfig.h            # 内核配置（中断优先级分区 / 堆 / API 开关）
├─ app/
│  ├─ Control/                 # 串级 PID + 混控 + 陀螺校准
│  ├─ Control_Task/            # RTOS 任务 + ISR 回调（6 任务调度）
│  ├─ PID/                     # PID 控制器
│  ├─ Filter/                  # 低通/互补滤波器
│  └─ My_Usart/                # 串口管理 + printf
├─ BSP/
│  ├─ MPU6050/                 # 六轴陀螺仪 + DMP
│  ├─ QMC5883P/                # 磁力计（含硬铁/软铁校准）
│  ├─ BMP280/                  # 气压计（含地面归零校准）
│  ├─ MTF02P/                  # 光流测距一体化（Micolink）
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
├─ Core/                       # 芯片底层实现（STM32F407）
├─ Drivers/                    # 启动文件 + CMSIS
├─ SYSTEM/                     # sys/Delay(DWT)/BusRate/IrqPriority
├─ Middlewares/FreeRTOS-Kernel/# FreeRTOS V11.1.0 内核源码（vendored）
├─ OpenOCD/                    # 下载配置
└─ docs/arch-guide.md          # 架构深度解析
```

## 🏗️ 飞控核心

```
遥控器 (NRF24L01, 100Hz)
     │ Key(1解锁/2锁定/3解锁+预设基准) + 油门摇杆 + Altitude_Stick_Input(高度)
     │ + Position_X/Y_Stick_Input(水平移动)
     ▼
┌──────────┐    ┌──────────┐    ┌──────────┐
│ 角度环 PID │ → │ 角速度环 PID│ → │ 混控矩阵  │ → DShot_Write()
└──────────┘    └──────────┘    └──────────┘
   500Hz            500Hz        Key 状态 base, X 型四轴
Pitch/Roll 串级 │ 偏航角速度环 (gyro_z 单环, 消除自旋)
                │ 定高环 (100Hz: Altitude_Stick_Input→积分高度→高度外环→速率内环;
                │   Key==3 额外注入预设基准油门 ALT_DEV_OUT_MAX)
                │ 定点环 (100Hz: Position_X/Y 摇杆→速率积分→位置目标; 光流×高度→
                │   实测速度→积分→位置估计; 位置外环 P→速度内环 PI→倾角指令→
                │   Target_Pitch/Roll, 复用串级外环)

FreeRTOS 任务模型（6 任务，优先级 6→1）：
  TIM2 ISR ─信号量→ ControlTask(6)  500Hz 串级PID + 偏航角速度环 + 定高环 + 定点环 + 混控
  EXTI    ─信号量→ SensorTask(5)    200Hz 读 MPU6050 DMP/陀螺/加速度
  Mtf02pTask(4) 2ms  MTF-02P Micolink 协议解析
  RadioTask(3) 10ms  NRF24L01 遥控+遥测
  TelemetryTask(2) 100ms 串口打印
  LEDTask(1) 100ms LED 状态指示（断链绿色闪烁 / 解锁 RGB 交替 / 锁定红色闪烁）

传感器数据流：
  MPU6050 DMP (200Hz) → Pitch/Roll → 角度环
  MPU6050 Gyro (500Hz) → gyrox/gyroy → Pitch/Roll 角速度环
  MPU6050 Gyro (500Hz) → gyroz → 偏航角速度环（零偏校准 + 低通 + PID）
  MTF02P (~100Hz) → distance (mm) → 定高环（✅ 已实测；差分+低通估爬升速率）
  MTF02P (~100Hz) → flow_x/y (cm/s@1m) → 定点环（✅ 已实测，架构见 arch-guide §5.5）
```

## 🏆 开发进度（里程碑）

| 里程碑 | 状态 |
|--------|------|
| 🛰️ 2.4G 遥控通信（NRF24L01 双向遥测） | ✅ |
| 🚁 串级 PID 正常起飞 | ✅ |
| 🌀 偏航角速度环（gyro_z）消除自旋 | ✅ |
| 💡 LED 控制管理任务（状态指示） | ✅ |
| 🎚️ 定高环（MTF02P ToF）** | ✅ 2026-09-05 实测 |
| 🛸 定点环（MTF02P 光流位置+速度串级） | ✅ 2026-09-07 实测 |

## 🎯 中断优先级

| 优先级 | 中断源 | 理由 |
|:---:|------|------|
| 4 | USART1/2/3 | 异步 TX/RX（不感知，永不被内核屏蔽） |
| 5 | TIM2 (2ms) | 控制节拍 → 信号量唤醒 ControlTask (500Hz) |
| 6 | MPU6050 EXTI | DMP 数据就绪 → 信号量唤醒 SensorTask (200Hz) |
| 15 | SysTick / PendSV | FreeRTOS 内核独占（tick + 上下文切换） |

## ⚙️ 构建与烧录

| 快捷键 | 功能 |
|--------|------|
| `F7` | 编译（Debug 预设） |
| `F8` | 烧录 |

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## 📖 详细文档

- 📐 工程架构深度解析：[docs/arch-guide.md](docs/arch-guide.md)
- 🧩 架构框架：[OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git)

## ⚠️ 注意事项

- 🎯 目标 MCU：STM32F407VET6（唯一维护目标，专注 F407 + FreeRTOS）
- 🗄️ 多平台移植（F103 / MSPM0G3507）已归档至 `git tag archive/multi-mcu-f103-g3507`，主分支不再保留
- 🚁 DShot 电调需从最低油门（48）逐步递增，不可直接跳到大油门值
- 🧘 上电后需保持飞行器静止 ~5 秒（陀螺 X/Y/Z 三轴零偏校准）
- 🔑 遥控状态由 Key（RxPacket[0]）切换：1=解锁（油门摇杆 speed_temp 打底 + 定高环偏差修正）；2=锁定停机；3=解锁+预设基准（base = ALT_DEV_OUT_MAX + 偏差输出，PID 只出修正不累）
- 🎚️ 定高环唯一宏 ALT_DEV_OUT_MAX（Control.h）= 预设基准油门（约悬停油门 600，实测 400 飞不起来），改这一处即改基准；其余参数（节拍/死区/限幅/抗扰）在 Control.h 的 Alt_Cfg_t 结构体；PID 增益与别的环一致在 main.c 用 Set_PID 调参
- 🛸 定点环参数在 Control.h 的 Pos_Cfg_t 结构体（节拍/高度窗 0.1~4.0m/满杆 1.5m/s/倾角限幅 12°/死区/限幅/抗扰/方向 dir_x、dir_y）；dir_x/dir_y 默认 +1，台架推杆测试方向反了就翻号；PID 增益在 main.c Set_PID 调参（位置外环 kp=1.0 纯 P / 速度内环 kp=8.0/ki=0.5）
- ⚠️ 空中 Key 1↔3 切换会使 base 阶跃 ±ALT_DEV_OUT_MAX，请勿空中切换；Key==1 时油门摇杆 speed_temp 参与打底，Key==3 由预设基准替代
- ⏱️ 软件 I2C/SPI 的 µs 级时序用 Delay(DWT)；任务 ms 级休眠用 vTaskDelay（详见 arch-guide §5.10）
- 🚦 只有优先级 ≥5 的中断可调用 FreeRTOS FromISR API；USART 回调内严禁调用
- 📡 MTF-02P 数据语义：distance (mm) 为 0 表示不可用；光流速度单位 cm/s@1m，实际速度 = 光流速度 × 高度(m)；定高定点前先查 tof_status / flow_status

## 📮 联系

- 📧 QQ 邮箱：634591772@qq.com
