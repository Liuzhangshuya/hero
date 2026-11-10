# 项目整理笔记（组件-功能-文件）

## 1. 项目定位与总体架构
- 项目类型：STM32F407（RM 机器人控制工程），基于 HAL + FreeRTOS。
- 控制对象：底盘、云台、发射机构（拨弹/摩擦轮）、传感器（IMU）、遥控链路、裁判系统、超级电容。
- 分层思路：
  - `Src/` + `Inc/`：CubeMX 生成的外设初始化与系统入口。
  - `application/`：业务控制与任务逻辑（底盘/云台/发射/感知/通信）。
  - `bsp/boards/`：板级驱动封装（CAN/USART/SPI/ADC/LED/激光/蜂鸣器等）。
  - `components/`：通用算法与设备中间层（PID、AHRS、BMI088、OLED、CRC/FIFO 等）。
  - `Drivers/`、`Middlewares/`：ST 官方 HAL/CMSIS、中间件。
  - `supercap_sdk/`：超级电容 CAN 通信 SDK。

---

## 2. 启动与调度主线

### 2.1 启动入口
- 主入口：`Src/main.c`
  - 完成外设初始化（GPIO/DMA/CAN/I2C/SPI/TIM/UART/USB 等）
  - 用户初始化链：
    - `can_filter_init()`
    - `delay_init()`
    - `cali_param_init()`
    - `remote_control_init()`
    - `usart1_tx_dma_init()`
    - `AUTO_control_init()`
    - `initSTValue()`
  - 调用 `MX_FREERTOS_Init()` 创建线程并启动调度器。

### 2.2 FreeRTOS 线程创建（当前工程状态）
- 线程定义文件：`Src/freertos.c`
- 已创建（未注释）：
  - `test_task`
  - `calibrate_task`
  - `chassis_task`
  - `gimbal_task`
  - `INS_task`
  - `led_RGB_flow_task`
  - `oled_task`
  - `battery_voltage_task`
- 已注释（当前未创建）：
  - `detect_task`
  - `referee_usart_task`
  - `usb_task`
  - `servo_task`
  - `PC_vosion_task`

> 注：`detect_task` 与 `referee_usart_task` 在业务上仍被大量依赖（`detect_hook`、`toe_is_error`、裁判数据），若确实未启用，运行行为会依赖默认值与其他路径，建议实机确认。

---

## 3. 组件-功能-文件映射（核心）

| 组件 | 主要功能 | 关键文件 |
|---|---|---|
| 系统入口/调度 | 芯片初始化、外设初始化、线程创建与启动 | `Src/main.c`、`Src/freertos.c` |
| 底盘控制 | 底盘模式切换、运动解算、PID 控制、电流下发 | `application/chassis_task.c`、`application/chassis_behaviour.c`、`application/chassis_task.h` |
| 底盘功率控制 | 基于裁判/超电反馈进行功率限幅与电流缩放 | `application/chassis_power_control.c`、`application/chassis_power_control.h` |
| 云台控制 | 云台模式管理、姿态闭环、yaw/pitch 电流控制 | `application/gimbal_task.c`、`application/gimbal_behaviour.c` |
| 发射机构 | 射击状态机、拨弹控制、摩擦轮控制、卡弹处理 | `application/shoot.c`、`application/shoot.h` |
| CAN 通信 | 电机反馈接收、设备在线检测钩子、控制帧发送（底盘/云台/摩擦轮/超电） | `application/CAN_receive.c`、`application/CAN_receive.h` |
| 遥控输入 | SBUS 协议解析、DMA + 空闲中断、掉线重启 | `application/remote_control.c`、`application/remote_control.h` |
| 裁判系统协议解析 | 按 cmd_id 解包并更新比赛/功率/热量等结构体 | `application/referee.c`、`application/referee.h`、`application/protocol/protocol.h` |
| 裁判串口接收任务 | USART6 数据入 FIFO、协议解包状态机 | `application/referee_usart_task.c` |
| IMU 姿态解算 | BMI088/IST8310 采样、温控、AHRS 融合输出欧拉角 | `application/INS_task.c` |
| 故障检测 | 设备离线/异常检测、优先级、恢复钩子 | `application/detect_task.c`、`application/detect_task.h` |
| 标定系统 | 云台/陀螺仪/底盘标定，参数持久化 Flash | `application/calibrate_task.c` |
| 自动瞄准通信 | 与上位机视觉串口通信、数据解包、弹道参数更新 | `application/AutoGimbal.c`、`application/AutoGimbal.h`、`application/SolveTrajectory.h` |
| USB 视觉服务 | USB CDC 队列接收视觉包并解包 | `application/Service_USB.c` |
| 电源监测 | ADC 采样电压、电量百分比估算 | `application/voltage_task.c` |
| 状态显示 | RGB 流水灯、OLED 故障/电量显示、USB 文本状态 | `application/led_flow_task.c`、`application/oled_task.c`、`application/usb_task.c` |
| 测试/告警 | 蜂鸣器按错误码报警 | `application/test_task.c` |

---

## 4. 底层与中间件目录说明

### 4.1 板级驱动（BSP）
- 目录：`bsp/boards/`
- 代表文件：
  - 通信：`bsp_can.*`、`bsp_usart.*`、`bsp_spi.*`、`bsp_i2c.*`
  - 传感与电源：`bsp_adc.*`、`bsp_rng.*`、`bsp_imu_pwm.*`
  - 执行器/外设：`bsp_fric.*`、`bsp_laser.*`、`bsp_led.*`、`bsp_buzzer.*`、`bsp_servo_pwm.*`
  - 工具：`bsp_delay.*`、`bsp_flash.*`、`bsp_crc32.*`

### 4.2 组件层（components）
- `components/controller/pid.*`：PID 控制器。
- `components/algorithm/`：AHRS 与数学库适配（含 `AHRS_middleware.*`）。
- `components/devices/`：BMI088、IST8310、OLED 驱动。
- `components/support/`：CRC、FIFO、内存管理等基础工具。

### 4.3 官方库与中间件
- `Drivers/CMSIS`、`Drivers/STM32F4xx_HAL_Driver`：内核/HAL。
- `Middlewares/ST`：USB、RTOS 相关中间件。
- `Middlewares/Systemview`：SEGGER SystemView 相关支持。

---

## 5. 通信链路速览

### 5.1 CAN
- 接收入口：`HAL_CAN_RxFifo0MsgPendingCallback()`（`application/CAN_receive.c`）
  - CAN1：底盘电机 + 摩擦轮等
  - CAN2：云台电机 + 顶部电机 + 超电反馈
- 发送接口：
  - `CAN_cmd_chassis(...)`
  - `CAN_cmd_gimbal(...)`
  - `CAN_cmd_fri(...)`
  - `CAN_cmd_SuperCap(...)`（通过 SDK 控制数据打包）

### 5.2 遥控/串口
- 遥控：`USART3_IRQHandler` + DMA 双缓冲解析 SBUS（`remote_control.c`）。
- 裁判：USART6 接收入 FIFO，再按协议状态机解包（`referee_usart_task.c` + `referee.c`）。
- 视觉：USART1/USB CDC 两套路径，均有包校验与结构化解包逻辑（`AutoGimbal.c`、`Service_USB.c`）。

---

## 6. 超级电容（supercap_sdk）集成点
- SDK 文件：
  - `supercap_sdk/include/supercap_sdk.h`
  - `supercap_sdk/src/supercap_sdk.c`
- 文档：`supercap_sdk/README.md`
- 当前集成位置：
  - `application/chassis_power_control.c` 中调用 `SuperCap_InitDefaultControl`、`CAN_cmd_SuperCap`，并读取 `SuperCap_Feedback` 参与功率上限计算。

---

## 7. 构建与工程文件
- Keil 主工程：`MDK-ARM/standard_robot.uvprojx`
  - 目标芯片：`STM32F407IGHx`
  - 生成目标：`standard_tpye_c`（含 hex）。
- VS Code 任务（工作区显示）：
  - `C/C++: gcc.exe 生成活动文件`（更偏单文件编译，不等价于 Keil 全工程构建）。
- CubeMX 配置文件：`standard_robot.ioc`

---

## 8. 维护建议（必要信息）
- 优先阅读顺序（新人上手）：
  1. `Src/main.c` → `Src/freertos.c`
  2. `application/chassis_task.c`、`application/gimbal_task.c`、`application/shoot.c`
  3. `application/CAN_receive.c`、`application/remote_control.c`
  4. `application/INS_task.c`、`application/detect_task.c`
  5. `application/chassis_power_control.c`、`supercap_sdk/README.md`
- 关注点：
  - `freertos.c` 中若禁用 `detect_task/referee_usart_task`，需评估对故障检测与功率/热量约束的影响。
  - `test_task` 在 `freertos.c` 有 `__weak` 版本，在 `application/test_task.c` 有正式实现，编译链接需确保行为符合预期。
  - 超电逻辑与裁判功率数据来源存在耦合，实车调试应重点核验 CAN ID、频率和掉线策略。

---

## 9. 快速索引（常用入口）
- 控制主循环：`application/chassis_task.c`、`application/gimbal_task.c`
- 功率控制：`application/chassis_power_control.c`
- CAN 中断：`application/CAN_receive.c`
- 遥控解析：`application/remote_control.c`
- IMU 解算：`application/INS_task.c`
- 裁判协议：`application/referee.c`、`application/protocol/protocol.h`
- 工程入口：`Src/main.c`、`Src/freertos.c`、`MDK-ARM/standard_robot.uvprojx`
