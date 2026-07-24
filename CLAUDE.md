# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

RoboMaster 哨兵机器人固件，基于 **STM32H750xx** (Cortex-M7, 单精度 FPU) 微控制器。项目包含两个 MCU 变体：

- **Top（云台/上层）** — 云台控制（ Pitch/Yaw 轴）、拨弹/摩擦轮射击控制、与上位机（Brain）的视觉通信
- **Base（底盘）** — 全向/麦轮底盘运动控制、超级电容管理、裁判系统读取、通过板间 CAN 与 Top 通信

两个项目共享相同的 HAL 层和 CubotMiddleware 架构，但**大部分 CubotMiddleware 文件在两个变体之间有差异**（不同步复制的），分别在 `Code/Top/Sentry_New/` 和 `Code/Base/Sentry_New/` 下独立维护。

## 构建与开发

### IDE & 工具链
- **Keil MDK-ARM** (μVision)，项目文件：`Code/{Top,Base}/Sentry_New/MDK-ARM/H7_Base.uvprojx`
- **ARM Compiler 6** (基于 Clang/LLVM，ARMCLANG)，参见 `Code/.clangd` 中的编译参数
- **STM32CubeMX** 生成 HAL 初始化代码：`Code/{Top,Base}/Sentry_New/H7_Base.ioc`

### 构建
1. 在 Keil μVision 中打开对应的 `.uvprojx` 文件
2. 选择目标构建配置，构建（F7）
3. 通过调试器（DAP-Link）烧录到对应 MCU

### clangd 语言服务器
`Code/.clangd` 为 IDE 代码智能提示配置了 ARM 编译参数。包含 CubeMX 生成外设的相对路径和绝对路径。**注意：** 如果仓库克隆到不同路径，需要更新 `.clangd` 中的绝对路径，以及 ARM 工具链的系统路径。

### 调试输出
固件通过 UART7 DMA 发送调试数据（使用 `UsartDmaPrintf` 函数），常配合 **Vofa+** 串口示波器使用。数据格式为 CSV 文本流。

### 编码
源文件使用 **GB2312** 编码（参见 `Code/.vscode/settings.json`）。中文注释普遍存在。

## 架构

### 分层结构

```
Src/Inc/                    ← STM32CubeMX 生成的 HAL 外设驱动（通过 .ioc 修改，不要手动编辑）
CubotMiddleware/
├── Drivers/                ← HAL 之上的薄封装层（CAN, UART, SPI, I2C, GPIO, Timer, Counter）
├── Devices/                ← 设备驱动：电机、IMU、遥控器、裁判系统等
├── Modules/                ← 应用模块：云台、底盘、射击、INS、通信
├── Algorithms/             ← PID、卡尔曼滤波、EKF 四元数姿态解算、数学工具
└── Support/                ← FIFO 缓冲区、Linux 风格链表
User/
├── hardware_config.c/h     ← 将所有外设和模块连接在一起的初始化入口
└── control_logic.c/h       ← 定时器回调任务、CAN 中断回调路由
```

### 执行模型

**裸机运行**（实际未使用 RTOS，尽管 `sys.h` 引用了 FreeRTOS 头文件）。所有实时工作通过以下方式驱动：

- **TIM14** @ 1 kHz — 主控制循环（云台控制、射击状态机、底盘控制、CAN 发送、调试打印输出）。任务函数：`TIM14_Task()`
- **TIM13** — IMU 读取（通过 I2C 读取 BM088/MPU6050，姿态解算）。任务函数：`TIM13_Task()`
- **CAN Rx FIFO 中断** — 电机状态反馈、板间通信。回调函数：`CAN1_rxCallBack()`、`CAN2_rxCallBack()`
- **UART 空闲中断** — 上位机视觉数据、激光雷达数据、裁判系统、遥控器（DR16/ET08）

主 `main()` 函数除了初始化外设外不做任何事——所有逻辑都在中断回调中运行。

### 关键中间件模式

#### CAN & 电机
- `CAN_Object` 封装 FDCAN 外设，维护已注册设备的链表
- `Motor` 结构体包含电机参数、运行时数据、`CAN_FillMotorData` / `Motor_DataUpdate` 函数指针，实现类似于虚函数的多态行为
- 支持的电机：**M3508**（底盘驱动）、**GM6020**（云台/转向）、**M2006**（拨弹盘）、**DM 电机**（达妙电机，MIT 协议）、**LK 电机**（凌控电机）
- CAN1 承载高优先级外设（电机 + 云台）；CAN2 承载低优先级外设 + 板间通信
- 发送 ID `0x1FF` / `0x200` 用于 DJI 智能电机控制，`0x1FE` 用于底盘

#### 双环 PID
- `DualPID_Object`：外层 PID（角度环）的输出作为内层 PID（角速度环）的目标值
- 用于云台的 Pitch/Yaw 轴和转向舵轮
- PID 参数在 `hardware_config.c` 中全局声明和初始化

#### INS（惯性导航系统）
- 基于 EKF 的四元数姿态估计（`quaternions_EKF`）
- 支持 IMU 传感器抽象：`IMU_InitData_t` 包含函数指针（`.Init`、`.Read`），兼容 BMI088 和 MPU6050
- 坐标系变换工具（Body frame ↔ Earth frame）

#### Brain 通信
- UART 双向协议（`Brain_t` 结构体），处理帧类型：`BRAIN_TO_ROBOT_CMD`（上位机→下位机指令）、`BRAIN_TO_ROBOT_HINT`（提示）、`ROBOT_TO_BRAIN_*`（下位机→上位机响应）
- 两个独立的 UART 通道：视觉自瞄（`Brain_Autoaim`）和激光雷达（`Brain_Lidar`）
- 上位机向下位机发送目标角度增量（`Pitch_add`、`Yaw_add`）和射击指令（`IsFire`）

#### 板间通信
- Top 和 Base 之间通过 CAN 通信，传输裁判系统数据、遥控器数据和同步信息
- `interboard.h` 定义了 `Top_t` 结构体（裁判系统数据），`communication.h` 定义了 `Receive_t`（聚合 Top/Base 数据）

### 内存布局（链接脚本）

```
Flash:  0x08000000 (128 KB) — 代码、只读数据
DTCM:   0x20000000 (128 KB) — 快速 RW/ZI 数据
AXI SRAM: 0x24000000 (512 KB) — 额外 RW/ZI 数据，通过 MPU 配置为可缓存
```

### 关键外设映射

| 外设 | 用途 |
|------|------|
| FDCAN1 | 高优先级 CAN：电机控制、DM 电机、云台 |
| FDCAN2 | 低优先级 CAN：附加电机、板间通信 |
| USART1 | 遥控器（DR16/ET08） |
| USART2 | 上位机视觉自瞄 |
| USART4 | 裁判系统 / 激光雷达 |
| USART6 | 板间 UART |
| USART7 | 调试输出（Vofa+） |
| SPI1 | BMI088 IMU |
| I2C2 | MPU6050 / 其他 I2C 设备 |
| TIM14 | 1 kHz 主控制循环 |
| TIM13 | IMU 读取循环 |

## 分支说明

- `main` — 分区赛代码基线
- `feat-dm-imu` — 当前分支，正在添加达妙 IMU 支持（`dmimu.c/h` 文件已创建但尚未实现）
- `feat-mg4005` — MG4005 电机支持

## 代码风格注意事项

- 中文变量名和注释在用户层代码中很常见（例如 `拨弹盘` = shoot plate，`云台` = gimbal）
- `holder` 指云台（gimbal），`swerving` 指舵轮底盘
- `Yaw_S` = Yaw 小轴（小齿轮），`Yaw_M` = Yaw 大轴（大齿轮）——Top 使用 Yaw_S，Base 使用 Yaw_M
- `AmmoBooster` = 包含摩擦轮和拨弹机构的射弹系统
- `et08` = ET08 遥控器，`dr16` = DR16 遥控器 ——互斥使用
- 舵轮底盘几何模型将平移速度（`Vx, Vy, Omega`）分解为 4 个独立转向模块的目标角度和转速
- 前馈 + PID 常见：`PitchFF_Gravity()` 补偿重力力矩，超级电容功率限制采用功率预算缩放

## CAN_2_UART 子项目

`Code/CAN_2_UART/Sentry_T2B/` 是一个**独立的 Keil MDK 工程**（非主固件的一部分），实现 CAN↔UART 双向协议转换桥接。

### 概述

- **MCU**: STM32F103C8T6 (Cortex-M3, 64MHz, 64KB Flash, 20KB SRAM)
- **编译器**: ARMCC V5.06 (ARM Compiler 5)，**非** ARMCLANG
- **CubeMX**: 6.18.0, Firmware Package STM32Cube FW_F1 V1.8.7
- **功能**: 将 CAN 总线帧与 UART 串口帧双向透明转发，用于哨兵机器人板间通信适配

### 硬件与外设

| 外设 | 引脚 | 用途 |
|------|------|------|
| CAN1 | PA11(RX), PA12(TX) | CAN 总线，1 Mbps，ABOM 自恢复使能 |
| USART3 | PB10(TX), PB11(RX) | UART，500,000 bps 8N1，DMA 收发 |
| DMA1_CH2 | — | USART3 TX（Memory→Peripheral, Normal 模式） |
| DMA1_CH3 | — | USART3 RX（Peripheral→Memory, Normal 模式） |

NVIC 优先级：DMA1_CH2/CH3(0) > USART3(1) > CAN1_RX0(2) > SysTick(15)

### 通信协议

固定 **9 字节帧**，双向对称：

```
Byte 0:      通道 ID
Bytes 1-8:   8 字节 payload
```

**通道映射**（全双工，收发通道不重叠）：

| 方向 | UART 通道 ID | CAN StdID |
|------|-------------|-----------|
| UART → CAN | 1 | 0x101 |
| UART → CAN | 2 | 0x102 |
| UART → CAN | 3 | 0x103 |
| UART → CAN | 4 | 0x104 |
| CAN → UART | 5 | 0x105 |
| CAN → UART | 6 | 0x106 |

CAN 硬件过滤器：16 位 ID 列表模式，仅接收 0x105、0x106（CAN→UART 方向）。UART→CAN 方向由 UART 侧按通道 ID 1~4 发起。

### 架构

```
CUBOT/
├── fifo.c/h           ← 消息 FIFO 循环缓冲区（临界区保护，离线检测）
├── user_can.c/h       ← CAN 初始化、CAN→UART 转发（DMA 链式发送）
└── user_usart.c/h     ← UART 初始化、UART→CAN 转发（重试 + bus-off 恢复）

Core/
├── Src/main.c         ← 入口，while(1) 中调用两个 TryTransmit 重试函数
├── Src/can.c          ← CubeMX 生成：CAN1 初始化
├── Src/usart.c        ← CubeMX 生成：USART3 DMA 初始化
├── Src/dma.c          ← CubeMX 生成：DMA 控制器初始化
└── Src/stm32f1xx_it.c ← 中断向量：CAN_RX0、USART3、DMA1_CH2/CH3
```

### 执行模型

裸机运行，所有工作由中断驱动 + main 循环兜底：

- **CAN RX FIFO0 中断** → `HAL_CAN_RxFifo0MsgPendingCallback` → 消息 Push 到 `fifo_can2uart` → 尝试 UART DMA 发送
- **UART 空闲中断** → `HAL_UARTEx_RxEventCallback` → 消息 Push 到 `fifo_uart2can` → 尝试 CAN mailbox 发送
- **DMA TX 完成中断** → `HAL_UART_TxCpltCallback` → Pop 下一条消息，链式 DMA 发送直到 FIFO 空
- **main `while(1)`** → 持续调用 `UART2CAN_TryTransmit()` 和 `CAN2UART_TryTransmit()` 作为失败重试兜底

### FIFO 缓冲与离线重发

为应对对端离线或瞬时负载峰值，两个方向各有一个 16 条 × 9 字节的 `MsgFIFO_t` 循环缓冲区：

**CAN→UART 方向**（`user_can.c`）：
- CAN 消息到达后 Push 到 `fifo_can2uart`
- `CAN2UART_TryTransmit()` 检查 DMA 空闲后 Pop 并启动传输
- DMA 完成回调链式触发下一条，无需 main 参与（main 调用仅作安全网）

**UART→CAN 方向**（`user_usart.c`）：
- UART 帧到达后 Push 到 `fifo_uart2can`
- `UART2CAN_TryTransmit()` 检查 mailbox 空闲后逐条发送
- 检测 CAN bus-off（读 `CAN_ESR_BOFF`），恢复后自动排空积压数据
- 连续发送失败 ≥8 次 → 标记离线 → 100ms 后自动重试
- FIFO 满时丢弃最旧消息，通过 UART 发送 `0xFF 0xEE...` 错误通知

**DMA 协调**：`uart_tx_dma_busy` 标志跨模块共享（`user_can.h` 声明 extern），UART→CAN 方向的错误通知仅在 DMA 空闲时发送，避免与 CAN→UART 数据流竞争 UART TX DMA。

### 构建

1. 打开 `Code/CAN_2_UART/Sentry_T2B/MDK-ARM/Sentry_T2B.uvprojx`
2. Keil μVision 中 Rebuild（F7）
3. 通过 SWD（PA13/PA14）烧录到 STM32F103C8T6

**注意**：主项目的 `.clangd` 配置针对 STM32H750 + ARMCLANG，与 CAN_2_UART 子项目（STM32F103 + ARMCC）不兼容。在 CAN_2_UART 目录下编辑时 clangd 可能报错，以 Keil MDK 编译结果为准。
