# LowComputer-Sentry

RoboMaster 2023 赛季哨兵电控代码开源项目

![哨兵](attachment/2023-07-10-14-02-20.png)

## 项目简介

本项目是基于 STM32F407 的 RoboMaster 赛季哨兵机器人下位机控制程序，涵盖了底盘运动控制、云台姿态控制、发射系统控制、视觉通信、裁判系统解析、IMU 姿态解算与电源管理等核心功能。项目采用 Keil MDK 进行开发，结合 FreeRTOS 任务管理，实现了哨兵机器人在赛场中的稳定控制与高效协同。

该工程是一个典型的比赛机器人控制程序，适合用于学习：

- STM32 嵌入式开发
- CAN 总线通信与电机控制
- 自主导航与机械臂/云台控制逻辑
- 机器人任务拆分与 RTOS 设计
- 视觉与裁判系统协议解析

## 主要功能

- 底盘运动控制：前后左右平移、旋转、摆尾、自动移动等
- 云台控制：Yaw/Pitch 角度控制、编码器校准、IMU 反馈控制
- 发射系统：摩擦轮控制、拨弹盘控制、卡弹检测、热量管理
- 视觉通信：接收视觉目标信息并输出控制指令
- 裁判系统：读取比赛状态、血量、热量等比赛数据
- 传感器融合：BMI088/IMU、陀螺仪和编码器结合解算姿态
- 设备管理：电流、电压、LED、OLED、蜂鸣器等外设控制
- 任务调度：基于 FreeRTOS 的多任务协同工作

## 硬件平台

- MCU：STM32F407IGH6
- 开发环境：Keil MDK / STM32CubeMX 生成工程
- 通信：CAN、UART、USART、SPI、I2C
- 控制模块：机械臂/云台电机、底盘电机、拨弹盘与摩擦轮电机
- 传感器：IMU、编码器、遥控器、裁判系统接口等

## 目录结构

```text
LowComputer-Sentry/
├── application/          # 业务逻辑任务：底盘、云台、发射、检测等
│   ├── CAN_receive.*
│   ├── chassis_task.*
│   ├── gimbal_task.*
│   ├── shoot_task.*
│   ├── detect_task.*
│   ├── vision_task.*
│   ├── referee_usart_task.*
│   └── ...
├── bsp/                  # 板级支持包：GPIO、USART、CAN、PWM、ADC、I2C 等
│   └── boards/
├── components/           # 通用算法与驱动模块
│   ├── algorithm/
│   ├── controller/
│   ├── devices/
│   └── support/
├── Drivers/              # ST 官方 HAL 驱动及 CMSIS
├── Inc/                  # HAL/外设头文件
├── MDK-ARM/              # Keil 工程文件
├── Middlewares/          # 中间件与 RTOS 相关文件
├── Src/                  # 中断与主程序入口
├── stm32/                # STM32 配置相关代码
├── attachment/           # 图片、文档资源
├── standard_robot.ioc    # CubeMX 工程配置
├── keilkilll.bat         # Keil 清理脚本
├── LICENSE               # MIT License
├── readme.md             # 项目说明
└── .mxproject            # STM32CubeMX 工程文件
```

## 核心模块说明

### 1. application

业务层主要包含各类控制任务：

- `chassis_task`：底盘电机控制、姿态跟随、自动移动
- `gimbal_task`：云台角度控制、编码器校准、IMU 控制
- `shoot_task`：摩擦轮、拨弹盘、电机闭环及卡弹处理
- `detect_task`：状态检测与故障监测
- `referee_usart_task`：裁判系统串口通信
- `vision_task`：视觉识别和控制指令处理
- `remote_control`：遥控器输入解析

### 2. bsp

板级驱动层主要包括：

- CAN、USART、GPIO、SPI、I2C、PWM、ADC、DAC 等外设初始化
- LED、Buzzer、OLED 等用户设备接口
- 电机接口、传感器接口和底层驱动封装

### 3. components

这里存放通用控制算法和设备 support：

- PID 控制器
- AHRS / 姿态解算
- 卡尔曼滤波
- CRC 校验
- 通信协议与数据处理

## 开发环境

本项目在 Windows 环境下使用 Keil MDK 进行编译和调试，代码编辑时也可结合 VS Code 等编辑器进行开发。

建议使用：

- Keil μVision 5
- STM32F4 系列开发板/定制机器人电控板
- J-Link / ST-Link 调试器
- VS Code（代码阅读、编辑、Git 管理）

## 快速上手

### 1. 打开工程

在 `MDK-ARM/` 目录中打开：

```text
standard_robot.uvprojx
```

### 2. 编译工程

使用 Keil 编译项目，确认依赖库已完整配置。若环境中缺少 `Middlewares` 或官方驱动，需确保工程中相关路径未丢失。

### 3. 下载程序

通过 ST-Link / J-Link 将编译完成的程序烧录至目标 STM32F407 板载 MCU。

### 4. 功能验证

常见调试步骤：

- 检查 CAN 总线通讯状态
- 验证遥控器输入是否正常
- 确认云台和底盘电机编码器方向
- 调整 PID 参数以稳定控制
- 测试发射、视觉、裁判系统之间的联动

## 开发建议

为了便于后续维护与二次开发，建议遵循以下约定：

- 控制任务尽量单独拆分到 `application/` 中
- 外设驱动统一放在 `bsp/boards/`，避免业务逻辑耦合底层寄存器
- PID 参数和控制策略集中管理，便于调参
- 关键逻辑增加注释，便于赛后调试与复盘
- 通过串口或 OLED 显示状态，方便现场定位异常

## 许可协议

本项目采用 MIT License，详情见 [LICENSE](LICENSE)。

## 备注

该代码来自 RoboMaster 赛季开发实践，适合用于学习嵌入式控制、机器人底层架构和实时控制策略。若你要在此基础上进行二次开发，请注意：

- 电机参数和机械结构参数因机器人差异而不同
- 视觉、发射和底盘参数需结合实际装配进行调试
- 任何修改都应优先在实验平台上进行实机验证

