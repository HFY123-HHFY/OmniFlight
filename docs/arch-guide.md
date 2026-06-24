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
│  Control/       串级PID + 混控 + 陀螺校准│
│  Control_Task/  中断回调 + 任务标志调度  │
│  PID/           PID 控制器              │
│  Filter/        低通/互补滤波器         │
│  IMU/           偏航角互补滤波融合       │
│  Altitude/      高度互补滤波融合         │
│  My_Usart/      串口管理 + printf       │
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  BSP/           板级支持层              │
│  MPU6050/ QMC5883P/ BMP280/            │  传感器驱动（含校准）
│  NRF24L01/      2.4G 无线（软件 SPI）   │
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
| I2C SCL | PB8 | 硬件 I2C1 | MPU/QMC/BMP 共用 |
| I2C SDA | PB9 | 硬件 I2C1 | |
| MPU6050 INT | PE7 | EXTI | DMP 数据就绪 (200Hz) |
| NRF SCK | PA5 | 软件 SPI | NRF24L01 |
| NRF MOSI | PA7 | 软件 SPI | |
| NRF MISO | PA6 | 软件 SPI | |
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

### 5.1 传感器数据流

```
┌─────────────────────────────────────────────────────────────────┐
│ TIM1 ISR (500Hz)                                                │
│   gyroz → IMU_Yaw_IntegrateGyro  陀螺积分更新偏航角               │
│   (PID_Pitch_Roll_Combined 待启用)                               │
├─────────────────────────────────────────────────────────────────┤
│ TIM2 ISR (1ms) → 任务标志调度                                    │
│   10ms → nrf_task_flag   (100Hz)                                │
│   20ms → qmc_task_flag   (50Hz)                                 │
│   50ms → bmp_task_flag   (20Hz)                                 │
│  100ms → print_task_flag (10Hz)                                 │
│ 1000ms → Timer_Bsp_t++   (1Hz 时间戳)                            │
├─────────────────────────────────────────────────────────────────┤
│ 主循环 — 消费任务标志                                             │
│   mpu_flag (200Hz)    → DMP Pitch/Roll + gyrox/gyroy/gyroz + aacz │
│   nrf_task_flag       → NRF24L01_Data() 遥控+遥测                │
│   qmc_task_flag       → Angle_XY=QMC_Data() → IMU_Yaw_CorrectMag │
│   bmp_task_flag       → alt=BMP_Data() → Altitude_Update(aacz,alt,0.05s) │
│   print_task_flag     → usart_printf 传感器数据                   │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 串级 PID 架构

```
目标角度 (遥控器) → 角度环 PID (外环 500Hz) → 角速度环 PID (内环 500Hz) → 混控 → DShot
```

- 外环：Pitch/Roll 角度 → 角速度目标（Out_max=±400°/s）
- 内环：gyrox/gyroy 去偏+低通 → 电机输出（Out_max=2047）
- 低通：alpha=0.45，截止频率 ~36Hz@500Hz

### 5.3 偏航角融合 (IMU)

MPU6050 DMP Yaw 存在长期漂移，改用互补滤波：

```
yaw += (gyro_z - bias) * dt        ← 陀螺积分 500Hz (TIM1 ISR)
yaw += Kp * (mag - yaw)            ← 磁力计校正 50Hz (主循环)
bias -= Ki * (mag - yaw)           ← 零偏在线补偿
```

- Kp=0.15, Ki=0.002
- 上电 5 秒自动采集 gyro_z 零偏
- 首次 QMC 数据直达起点，无收敛延迟
- 输出：`IMU_Yaw` 全局变量 (0~360°)，替代 DMP Yaw
- `IMU_Init()` 必须在 `API_TIM_Init(TIM1)` 之前调用

### 5.4 高度融合 (Altitude)

BMP280 气压计噪声 ±1m 且响应慢，加速度计 Z 轴积分短期精确但长期发散。互补滤波融合：

```
pos += vel * dt                         ← 速度积分 (20Hz)
vel += (aacz - gravity_ref) * G_SCALE * dt  ← 加速度积分
pos += Kp * (baro_alt - pos)            ← 气压计 P 校正
vel += Ki * (baro_alt - pos)            ← 速度零偏 I 校正
```

- Kp=0.4, Ki=0.2, 调用频率 20Hz（bmp_task_flag）
- `gravity_ref` 与陀螺零偏同步采集（同一次 5s 静止），无额外等待
- 输出：`Alt_Fused` — 融合高度 (m)，钳位 >= 0
- `Altitude_Init()` 在 `GyroBias_Calibrate` 之后调用，接收重力参考值

### 5.5 传感器校准

| 传感器 | 校准方式 | 耗时 | 说明 |
|--------|----------|:---:|------|
| MPU6050 Gyro X/Y + 重力参考 | 上电静止采集 1000 帧 | ~5s | `GyroBias_Calibrate(1000U, &gravity_ref)` 同步采集 |
| MPU6050 Gyro Z | IMU 互补滤波在线 PI | ~5s | TIM1 ISR 自动完成 |
| QMC5883P | 硬铁 offset + 软铁 scale (预存参数) | 瞬间 | `QMC_CAL_ENABLE=0` 使用头文件预存值 |
| BMP280 | 地面气压归零 + EMA 跟踪 | 5s | `BMP280Init` 内部自动执行 |

总启动时间：约 10 秒（陀螺+重力 5s + BMP 5s）

### 5.6 DShot300 油门协议

- TIM1 配置为 300kHz PWM 基波（ARR=560, PSC=1）
- DMA2 Stream5 burst 模式，每次 TIM1 溢出自动更新 CCR1~CCR4
- 4 路电机通过 X-Macro 映射到物理通道：CH1←m3, CH2←m1, CH3←m2, CH4←m4
- 油门范围：48~2047（0 停转，1~47 为保留命令区）
- **重要**：电调需要从最低油门（48）逐步递增，不可直接跳到大油门值

### 5.7 中断回调架构

- `Control_Task1_Callback` → TIM1 1ms → pid_2ms_tick (2ms/500Hz) → IMU 积分 + PID
- `Control_Task2_Callback` → TIM2 1ms → 所有任务标志调度
- `Control_Task_USART_Callback` → USART1/3 → TX 队列排空 + RX 接收

### 5.8 任务标志位一览

| 标志 | 频率 | ISR 源 | 主循环消费 |
|------|:---:|--------|------------|
| `mpu_flag` | 200Hz | MPU6050 EXTI | DMP + Gyro 读取 |
| `nrf_task_flag` | 100Hz | TIM2 | NRF24L01_Data() |
| `qmc_task_flag` | 50Hz | TIM2 | QMC_Data() → IMU_Yaw_CorrectMag() |
| `bmp_task_flag` | 20Hz | TIM2 | BMP_Data() → Altitude_Update() |
| `print_task_flag` | 10Hz | TIM2 | usart_printf |

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

### 6.3 I2C 总线设计

- 三个设备共享 I2C1（PB8/PB9）：MPU6050（0xD0）、QMC5883P（0x58）、BMP280（0xEC）
- 统一 400kHz Fast Mode
- 所有 I2C 读写均在主循环执行，ISR 不访问 I2C，避免硬件 I2C 状态机死锁
- `BMP280_SelectI2CSpeed()` 每次 I2C 事务前选择总线+速率（BMP280 20Hz，开销可忽略）

### 6.4 NRF24L01 软件 SPI

- 使用软件 SPI（非硬件 SPI1），100Hz 轮询足够
- 硬件 SPI 存在与硬件 I2C 相同的不可重入问题，主循环传输若被 ISR 打断会丢数据
- 无其他 SPI 设备竞争，软件 SPI 带宽（~4MHz）远超 NRF 需求

---

## 7. 中断优先级 (IrqPriority.h)

| 优先级 | 中断源 | 理由 |
|:---:|------|------|
| 0 | SysTick | 系统心跳基准 |
| 1 | TIM1 (API_TIM1) | 1ms 控制节拍，PID/IMU/混控核心 |
| 2 | MPU6050 EXTI | DMP 数据就绪，姿态外环输入 |
| 3 | TIM2 (API_TIM2) | 任务标志调度 / 时间戳 |
| 4 | USART1/3 | 通信（丢包可重传） |

---

## 8. BSP 模块清单

| 模块 | 依赖层 | 说明 |
|------|--------|------|
| LED | API_GPIO | Enroll 注册，校准状态指示 |
| MPU6050 | API_I2C + EXTI | DMP 姿态解算 + 陀螺原始值 |
| QMC5883P | API_I2C | 地磁航向（含硬铁/软铁校准） |
| BMP280 | API_I2C | 气压高度（含地面归零校准） |
| NRF24L01 | API_SPI + Enroll CE | 2.4G 遥控遥测（软件 SPI） |
| IMU | MPU6050 + QMC5883P | 偏航角互补滤波融合 |
| Altitude | MPU6050 + BMP280 | 高度互补滤波融合 (aacz + 气压计) |
| Dshot | Core f407_pwm + f407_dma | DShot300 油门 |
| Motor | Dshot + Control | 电机混控反饱和 |
| Buzzer | API_PWM | 无源蜂鸣器 |

---

## 9. 已知注意事项

1. **USART3 AF 修复**：`API_USART_GetAfNum` 从 AF8 改为 AF7（STM32F407 所有 USART 都是 AF7）
2. **NRF24L01 CE 注册**：`Enroll_NRF24L01_Register()` 必须在 `NRF24L01_Init()` 前调用
3. **DShot 电调**：油门值需从最低（48）逐步递增，电调才响应
4. **printf 异步 TX**：必须注册 USART 中断回调，否则 TX 队列不会排空
5. **F407 以外平台**：f407_dma、Dshot、Motor、Buzzer 仅在 F407 编译
6. **IMU_Init 顺序**：必须在 `API_TIM_Init(TIM1)` 之前调用，否则 ISR 访问未初始化状态
7. **上电静止**：飞行器需静止 ~10s（陀螺+重力 5s + BMP 5s）完成所有传感器校准
8. **I2C 总线**：所有 I2C 读写仅在主循环，ISR 不触碰 I2C（硬件 I2C 不可重入）
9. **BMP280 地面跟踪**：未解锁时 EMA 持续跟踪地面气压，解锁后冻结。飞完降落后**必须先锁定等 2s alt 归零**再重新解锁。

---

## 10. 快速恢复认知

1. [README.md](README.md) — 项目概览
2. 本文档 — 架构全貌
3. [A_Entry/main.c](A_Entry/main.c) — 飞控初始化 → 主循环
4. [Enroll/407_hw_config.h](Enroll/407_hw_config.h) — F407 板级映射
5. [app/Control/Control.c](app/Control/Control.c) — 串级 PID + 陀螺校准
6. [app/Control_Task/Control_Task.c](app/Control_Task/Control_Task.c) — 中断回调 + 任务调度
7. [app/IMU/IMU.c](app/IMU/IMU.c) — 偏航角互补滤波融合
8. [app/Altitude/Altitude.c](app/Altitude/Altitude.c) — 高度互补滤波融合
9. [BSP/Dshot/Dshot.c](BSP/Dshot/Dshot.c) — DShot300 协议
10. [SYSTEM/IrqPriority.h](SYSTEM/IrqPriority.h) — 中断优先级
11. [CMakeLists.txt](CMakeLists.txt) — 构建入口
