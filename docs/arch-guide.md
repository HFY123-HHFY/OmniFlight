# OmniFlight 工程架构深度解析

> 本文档用于 AI 上下文初始化，新对话加载此文档即可快速恢复对项目架构的完整认知。

---

## 1. 项目元信息

| 项目 | 详情 |
|------|------|
| **名称** | OmniFlight — 四轴飞控 |
| **基于框架** | [OmniLayer](https://github.com/HFY123-HHFY/OmniLayer.git) |
| **主控** | STM32F407VET6 (Cortex-M4 + FPU, 168MHz, 512KB Flash, 128KB RAM) |
| **RTOS** | FreeRTOS V11.1.0（vendored 至 `Middlewares/FreeRTOS-Kernel`，heap_4） |
| **构建工具** | CMake + GCC ARM Embedded + OpenOCD |
| **IDE** | VS Code (CMake + GCC + OpenOCD) |
| **默认 MCU** | `ENROLL_MCU_F407` (定义于 Enroll/Enroll.h) |
| **分支** | `main` (FreeRTOS 实时飞控主线) |

---

## 2. 分层架构

```
┌────────────────────────────────────────┐
│  A_Entry/main.c  程序入口               │  初始化 → vTaskStartScheduler()
│  A_Entry/FreeRTOSConfig.h 内核配置      │  中断优先级分区 / 堆 / API 开关
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  app/           应用层 — 飞控核心算法   │
│  Control/       串级PID(含偏航角速度环) │
│                 + 混控 + 陀螺校准       │
│  Control_Task/  RTOS 任务 + ISR 回调    │  6 任务调度 + 信号量/互斥锁
│  PID/           PID 控制器              │
│  Filter/        低通/互补滤波器         │
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
│                 + f407_dma (DMA 抽象)   │
└────────────────────────────────────────┘
              ↓
┌────────────────────────────────────────┐
│  Drivers/       驱动资源层              │  CMSIS + 启动文件
│  SYSTEM/        系统层                  │  sys/Delay(DWT)/BusRate/IrqPriority
├────────────────────────────────────────┤
│  Middlewares/FreeRTOS-Kernel/          │  内核源码（vendored，任务/队列/互斥锁）
└────────────────────────────────────────┘
```

---

## 3. 当前开发 MCU

| MCU | 状态 |
|-----|:---:|
| **STM32F407VET6** | ★ 唯一目标 |

> 架构通过 Enroll 注册层 + X-Macro 编译期映射实现算法与芯片解耦，本身可移植。
> F103 / MSPM0G3507 的移植曾用于验证该抽象，现已归档至 `git tag archive/multi-mcu-f103-g3507`；
> 毕设主线为聚焦 F407 + FreeRTOS 实时化，主分支仅保留 F407。

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
| USART1 TX/RX | PB6/PB7 | USART1 | 板载调试串口 115200 |
| USART2 TX/RX | PD5/PD6 | USART2 | 板载调试 115200 |
| USART3 TX/RX | PD8/PD9 | USART3 | 无线串口 115200 |
| USART4 TX/RX | PA0/PA1 | UART4 | MTF-02P 115200（AF8，向量名 UART4_IRQHandler） |
| LED1/2/3 | PE2/PE3/PE4 | GPIO | 绿/红/蓝 |

---

## 5. 飞控核心设计

### 5.1 RTOS 任务模型（接管原裸机控制链）

```
┌─────────────────────────────────────────────────────────────────────┐
│ TIM2 ISR (2ms, 优先级5) → xControlSem ──► ControlTask(6)            │
│   500Hz：读姿态快照 → 串级PID + 偏航角速度环 → 混控 → DShot          │
├─────────────────────────────────────────────────────────────────────┤
│ MPU6050 EXTI (200Hz, 优先级6) → xMpuSem ──► SensorTask(5)           │
│   200Hz：I2C 读 DMP + 陀螺 + 加速度 → 互斥锁提交姿态全局变量          │
├─────────────────────────────────────────────────────────────────────┤
│ Mtf02pTask(4)    2ms 轮询   → MTF02P_Task() Micolink 协议解析 (USART4 115200) │
│ RadioTask(3)     10ms 周期  → NRF24L01_Data() 遥控+遥测 (100Hz)      │
│ TelemetryTask(2) 100ms 周期 → usart_printf（xPrintMutex 保护）       │
│ LEDTask(1)      100ms 周期 → LED 状态机（最低应用优先级）             │
└─────────────────────────────────────────────────────────────────────┘
```

任务优先级 6（最高）→ 1（最低应用优先级），0 为 idle，软件定时器服务任务与 TelemetryTask 同为 2（时间片轮转）。抢占式调度：
ControlTask 由 TIM2 信号量唤醒后立即抢占所有低优先级任务，保证 500Hz 控制节拍抖动最小。

### 5.2 串级 PID 架构

```
目标角度 (遥控器) → 角度环 PID (外环 500Hz) → 角速度环 PID (内环 500Hz) → 混控 → DShot
Pitch/Roll 串级；偏航轴为单角速度环（见 §5.3）
```

- 外环：Pitch/Roll 角度 → 角速度目标（Out_max=±400°/s）
- 内环：gyrox/gyroy 去偏+低通 → 电机输出（Out_max=2047）
- 偏航：gyroz 去偏+低通 → 角速度 PID → 混控 yaw 项（Out_max=500，单环）
- 低通：Pitch/Roll alpha=0.45，Yaw alpha=0.20（截止约 20Hz@500Hz，三轴各一个实例）

### 5.3 偏航角速度环

偏航轴控制目标：消除自旋（目标角速度恒 0，`Target_Yaw_Rate` 全局变量供后续遥控写入）。
单角速度环（无角度外环），500Hz 在 ControlTask 内计算：

```
gyroz (LSB) → 去零偏 → deg/s → 低通 → PID (kp=2.0, ki=0.05) → 混控 yaw 项
```

- gyro_z 零偏在上电 `GyroBias_Calibrate()` 内与 X/Y 同步校准（1000 样本平均）
- PID：`pid_rate_yaw`，Out_max=500（偏航力矩靠反扭矩差，权限远弱于俯仰/滚转）
- 抗抖设计（台架实测：静止偶发单拍跳变 + 转动测试积分残留导致电机卡顿）：
  - 低通 alpha=0.20（截止约 20Hz@500Hz，比俯仰/滚转的 0.45 更狠，抗振动毛刺）
  - 角速度死区 3dps（PID 库 `PID_SetDeadband`，|rate|≤3 视为 0，积分同时冻结）
  - 输出死区 10（|yaw_out|≤10 清零），死区内每拍 error_sum×0.99 泄放积分
  - 积分分离阈值 30dps（转动测试时不积分，防残留）
- 混控：`M1/M3` 对角 +yaw，`M2/M4` 对角 −yaw，方向宏 `MOTOR_YAW_DIR`（Motor.c，
  台架验证后若阻力矩方向相反则翻符号；最终以试飞为准：开了比不开转得更快 → 翻符号）
- 解锁边沿 `Control_Arm_Reset` 同步重置 yaw PID 与低通
- 历史：曾用 QMC5883P 磁力计互补滤波融合 IMU_Yaw（app/IMU），因地磁干扰不可靠已删除；
  航向保持外环可后加（用 DMP 或陀螺积分短时航向，而非磁力计）

### 5.4 高度/位置（后续工作）

历史版本曾用 aacz + BMP280 互补滤波融合高度（app/Altitude），实测低空不可信已删除。
当前高度方案定为 MTF02P ToF 测距 `mtf02p_data.distance`（mm，低空有效），
位置方案定为光流 `flow_x/flow_y`（cm/s@1m，实际速度 = 光流速度 × 高度(m)），
定高定点环待后续开发。BMP280/QMC5883P 驱动保留（未初始化），供高空高度/航向复用。

### 5.5 传感器校准

| 传感器 | 校准方式 | 耗时 | 说明 |
|--------|----------|:---:|------|
| MPU6050 Gyro X/Y/Z | 上电静止采集 1000 帧 | ~5s | `GyroBias_Calibrate(1000U)` 三轴同步校准，Z 供偏航环去偏 |
| QMC5883P | 硬铁 offset + 软铁 scale (预存参数) | 瞬间 | 驱动保留，当前未初始化（磁干扰不可靠） |
| BMP280 | 地面气压归零 + EMA 跟踪 | 5s | 驱动保留，当前未初始化（低空不可靠） |

总启动时间：约 5 秒（陀螺三轴零偏）

> RTOS 说明：校准类函数在**调度器启动前**调用，采样节拍用 `Delay_ms`（DWT 忙等，
> 不与 RTOS tick 冲突），不再依赖 EXTI 标志位轮询。

### 5.6 DShot300 油门协议

- TIM1 配置为 300kHz PWM 基波（ARR=560, PSC=1）
- DMA2 Stream5 burst 模式，每次 TIM1 溢出自动更新 CCR1~CCR4
- 4 路电机通过 X-Macro 映射到物理通道：CH1←m3, CH2←m1, CH3←m2, CH4←m4
- 油门范围：48~2047（0 停转，1~47 为保留命令区）
- **重要**：电调需要从最低油门（48）逐步递增，不可直接跳到大油门值

### 5.7 ISR 回调架构

ISR 只做「给信号量 / 硬件搬运」，业务逻辑全部在任务里：

- `Control_Task1_Callback` → TIM2 2ms ISR → `xSemaphoreGiveFromISR` 唤醒 ControlTask（500Hz，定时器直接按 2ms 配置，ISR 内不分频）
- `ControlTask_NotifyMpuIsr` → MPU6050 EXTI ISR → `xSemaphoreGiveFromISR` 唤醒 SensorTask（200Hz）
- `Control_Task_USART_Callback` → USART1~4 → TX 队列排空 + RX 按串口分发（FreeRTOS 不感知，不调任何 RTOS API）；USART4 数据 → `MTF02P_RxPush` 入队，解析在 Mtf02pTask

### 5.8 RTOS 对象一览

| 任务 | 优先级 | 频率 | 唤醒方式 | 职责 |
|------|:---:|:---:|----------|------|
| ControlTask | 6 | 500Hz | xControlSem（TIM2 ISR） | 姿态快照 → 串级 PID（含偏航角速度环）→ 混控 |
| SensorTask | 5 | 200Hz | xMpuSem（EXTI ISR） | I2C 读 DMP/陀螺/加速度 → 提交全局姿态 |
| Mtf02pTask | 4 | 2ms 轮询 | xTaskDelayUntil | MTF02P_Task Micolink 协议解析（距离+光流） |
| RadioTask | 3 | 10ms | xTaskDelayUntil | NRF24L01_Data 遥控+遥测 |
| TelemetryTask | 2 | 100ms | xTaskDelayUntil | usart_printf 遥测打印 |
| LEDTask | 1 | 100ms | xTaskDelay | 链路、解锁、锁定状态 LED 指示 |

| 同步对象 | 类型 | 保护内容 |
|----------|------|----------|
| xControlSem | 二值信号量 | TIM2 → ControlTask 500Hz 节拍 |
| xMpuSem | 二值信号量 | EXTI → SensorTask 200Hz 数据就绪 |
| xAttitudeMutex | 互斥锁（优先级继承） | Pitch/Roll/Yaw/gyrox/y/z/aacx/y/z（SensorTask 写 / ControlTask 读） |
| xPrintMutex | 互斥锁 | printf 串口输出防交织 |

### 5.9 FreeRTOS 接管设计要点

- **启动流程**：main 初始化外设 → `Control_Task_RTOSInit()`（信号量/互斥锁/任务必须先于 ISR 源创建）→ `vTaskStartScheduler()` → SVC 启动第一个任务，SysTick/PendSV 移交内核
- **中断优先级分区**：`configMAX_SYSCALL_INTERRUPT_PRIORITY=5`。USART=4 设为「不感知」（永不被内核临界区屏蔽，异步 TX 零丢包，但严禁调 RTOS API）；TIM2=5、EXTI=6 为「感知」（可调 FromISR API）
- **同步设计**：ISR → 二值信号量 → 任务；共享姿态数据 → 互斥锁（优先级继承防优先级翻转）；printf → 互斥锁防交织
- **延时选型**：µs 级 bit-bang 时序（软件 I2C/SPI）用 `Delay_us`（DWT 忙等，任务切换会被阻塞）；任务 ms 级休眠用 `vTaskDelay/xTaskDelayUntil`（让出 CPU）；Delay 已改为 DWT 计数器实现，与 SysTick 解耦
- **安全钩子**：`configCHECK_FOR_STACK_OVERFLOW=2` + 栈溢出/内存分配失败钩子 → 关中断死循环，方便调试器定位

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
- RTOS 下 USART 优先级 4（不感知）：TX/RX 永不被内核临界区屏蔽，且不调 RTOS API，天然安全

### 6.3 I2C 总线设计

- 三个设备共享 I2C1（PB8/PB9）：MPU6050（0xD0）、QMC5883P（0x58）、BMP280（0xEC）
- 统一 400kHz Fast Mode
- 所有 I2C 读写仅在任务上下文执行（SensorTask 等），ISR 不访问 I2C，避免硬件 I2C 状态机死锁
- `BMP280_SelectI2CSpeed()` 每次 I2C 事务前选择总线+速率（BMP280 20Hz，开销可忽略）

### 6.4 NRF24L01 软件 SPI

- 使用软件 SPI（非硬件 SPI1），100Hz 轮询足够
- 硬件 SPI 存在与硬件 I2C 相同的不可重入问题，任务上下文传输若被更高优先级任务/ISR 打断会丢数据
- 无其他 SPI 设备竞争，软件 SPI 带宽（~4MHz）远超 NRF 需求

---

## 7. 中断优先级 (IrqPriority.h)

FreeRTOS 分区：优先级 5~15「感知」（可调 FromISR API），0~4「不感知」（严禁调 RTOS API）。

| 优先级 | 中断源 | 感知 | 理由 |
|:---:|------|:---:|------|
| 4 | USART1~4 | 不感知 | 异步 TX/RX 纯内存操作，永不被内核屏蔽 → 串口零丢包 |
| 5 | TIM2 控制节拍 | 感知 | `xSemaphoreGiveFromISR` → 唤醒 ControlTask（500Hz） |
| 6 | MPU6050 EXTI | 感知 | `xSemaphoreGiveFromISR` → 唤醒 SensorTask（200Hz） |
| 15 | SysTick / PendSV | 内核 | FreeRTOS 独占：系统节拍 + 上下文切换（固定最低） |

---

## 8. BSP 模块清单

| 模块 | 依赖层 | 说明 |
|------|--------|------|
| LED | API_GPIO | Enroll 注册，校准状态指示 |
| MPU6050 | API_I2C + EXTI | DMP 姿态解算 + 陀螺原始值 |
| QMC5883P | API_I2C + Delay | 地磁航向（含硬铁/软铁校准）— 驱动保留，当前未初始化 |
| BMP280 | API_I2C | 气压高度（含地面归零校准）— 驱动保留，当前未初始化 |
| NRF24L01 | API_SPI + Enroll CE | 2.4G 遥控遥测（软件 SPI） |
| MTF02P | 无（纯协议解析） | Micolink 光流测距一体化（USART4）：距离 mm + 光流 cm/s@1m + 质量状态 |
| Dshot | Core f407_pwm + f407_dma | DShot300 油门 |
| Motor | Dshot + Control | 电机混控反饱和（三轴 PID → X 型矩阵，含 yaw 对角项） |
| Buzzer | API_PWM | 无源蜂鸣器 |

---

## 9. 已知注意事项

1. **USART AF 复用号**：`API_USART_GetAfNum` 按串口返回 AF：USART1/2/3 = AF7，UART4 = AF8（UART5 同理）；中断向量名 UART4/UART5 无 'S'（`UART4_IRQHandler`），与 USART1~3 不同
2. **NRF24L01 CE 注册**：`Enroll_NRF24L01_Register()` 必须在 `NRF24L01_Init()` 前调用
3. **DShot 电调**：油门值需从最低（48）逐步递增，电调才响应
4. **printf 异步 TX**：必须注册 USART 中断回调，否则 TX 队列不会排空
5. **单平台**：全工程仅针对 STM32F407 编译；多平台移植（F103/G3507）见归档 tag `archive/multi-mcu-f103-g3507`
6. **RTOS 对象先于 ISR 源**：`Control_Task_RTOSInit()` 必须在 TIM2/EXTI 启动前调用，否则 ISR 给信号量时对象为 NULL
7. **FromISR 限制**：只有优先级 ≥5 的中断可调 `...FromISR()`；USART（优先级 4）回调内严禁调用任何 FreeRTOS API
8. **延时选型**：软件 I2C/SPI 的 µs 级 bit-bang 时序必须用 `Delay_us`（DWT 忙等）；任务级 ms 休眠用 `vTaskDelay/xTaskDelayUntil`
9. **上电静止**：飞行器需静止 ~10s（陀螺+重力 5s + BMP 5s）完成所有传感器校准
10. **I2C 总线**：所有 I2C 读写仅在任务上下文，ISR 不触碰 I2C（硬件 I2C 不可重入）
11. **BMP280 地面跟踪**：驱动保留未启用。若重新启用：未解锁时 EMA 持续跟踪地面气压，解锁后冻结。飞完降落后**必须先锁定等 2s alt 归零**再重新解锁
12. **控制节拍依赖 TIM2**：TIM2 停则 ControlTask 永不唤醒，飞行中调试断点勿停在 TIM2 ISR 内
13. **栈溢出钩子**：`configCHECK_FOR_STACK_OVERFLOW=2` 开启，任务栈不足会进入 `vApplicationStackOverflowHook` 死循环（关中断），用调试器看 `pcTaskName` 定位
14. **MTF-02P 数据语义**：distance (mm) 为 0 表示不可用；光流速度单位 cm/s@1m，实际速度 = 光流速度 × 高度(m)；定高定点前先查 `MTF02P_IsRangeValid()` / `MTF02P_IsFlowValid()`（对应 tof_status / flow_status）

---

## 10. 快速恢复认知

1. [README.md](README.md) — 项目概览
2. 本文档 — 架构全貌
3. [A_Entry/main.c](A_Entry/main.c) — 飞控初始化 → RTOS 调度器启动
4. [A_Entry/FreeRTOSConfig.h](A_Entry/FreeRTOSConfig.h) — 内核配置（中断优先级分区 / 堆 / API 开关）
5. [Enroll/407_hw_config.h](Enroll/407_hw_config.h) — F407 板级映射
6. [app/Control/Control.c](app/Control/Control.c) — 串级 PID（含偏航角速度环）+ 陀螺三轴零偏校准
7. [app/Control_Task/Control_Task.c](app/Control_Task/Control_Task.c) — 5 任务 + 信号量/互斥锁 + ISR 回调
8. [BSP/Motor/Motor.c](BSP/Motor/Motor.c) — 三轴混控（X 型矩阵 + yaw 对角项 + MOTOR_YAW_DIR）
9. [BSP/Dshot/Dshot.c](BSP/Dshot/Dshot.c) — DShot300 协议
10. [SYSTEM/IrqPriority.h](SYSTEM/IrqPriority.h) — 中断优先级（FreeRTOS 分区）
11. [Middlewares/FreeRTOS-Kernel/](Middlewares/FreeRTOS-Kernel/) — vendored 内核 V11.1.0
12. [CMakeLists.txt](CMakeLists.txt) — 构建入口
