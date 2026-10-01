# RCS_code — STM32F405 哨兵电控工程

本仓库是哨兵机器人电控端固件工程，运行于 STM32F405RG，使用 FreeRTOS、STM32 HAL、Visual Studio 与 VisualGDB 开发。

工程当前包含底盘、大小云台、Pitch、电机、裁判系统、遥控器、双 IMU，以及电控与视觉小电脑之间的 RMV1 串口通信代码。

> 当前视觉接口处于“通信与反馈联调”阶段：视觉可以接收电控状态、两颗 IMU 的原始姿态和三个云台关节状态，但还不能控制云台、底盘或发射机构。

## 1. 硬件与开发环境

- MCU：STM32F405RG，Cortex-M4；
- RTOS：FreeRTOS Kernel 10.3.1；
- HAL：STM32F4 HAL；
- IDE：Visual Studio 2019；
- 插件：VisualGDB；
- ARM 工具链：`com.visualgdb.arm-eabi`，工程记录版本 `10.3.1/10.2.90/r1`；
- 工程入口：`FreeRTOS.sln`；
- VisualGDB 配置：`Debug|VisualGDB`、`Release|VisualGDB`。

## 2. 目录和主要文件

```text
RCS_code/
├─ FreeRTOS.sln                     Visual Studio解决方案
├─ 给视觉同学的RMV1对接说明.md       视觉联调说明
└─ STM32F405/
   ├─ STM32F405.cpp                 外设与业务对象初始化
   ├─ taskslist.cpp/.h              FreeRTOS任务入口
   ├─ control.cpp/.h                底盘、云台及发射控制
   ├─ RC.cpp/.h                     遥控器解析与模式选择
   ├─ motor.cpp/.h                  普通CAN电机
   ├─ HTmotor.cpp/.h                DM电机通信与反馈
   ├─ imu.cpp/.h                    CH010/HI226 IMU解析
   ├─ xuc.cpp/.h                    RMV1视觉电控协议
   ├─ auto_command.h                视觉目标命令数据结构
   ├─ usart.cpp/.h                  串口与DMA
   ├─ can.cpp/.h                    CAN通信
   ├─ judgement.cpp/.h              裁判系统
   ├─ CRC.cpp/.h                    CRC工具
   └─ STM32F405.vcxproj             VisualGDB工程
```

`.vs/`、`.visualgdb/` 和 `VisualGDB/Debug`、`VisualGDB/Release` 等目录属于IDE索引、调试状态或编译产物，不是源码，克隆后可以重新生成。

## 3. 初始化中的主要串口

| 用途 | 对象 | 外设 | 波特率 | 当前协议 |
|---|---|---|---:|---|
| 大Yaw侧IMU | `imu_big_pantile` | UART5 | 115200 | CH010 |
| 小Yaw/Pitch侧IMU | `imu_small_pantile` | USART3 | 921600 | CH010 |
| 视觉小电脑 | `xuc` | USART6 | 460800 | RMV1 |

USART6 当前引脚映射：

```text
PC6 = USART6_TX
PC7 = USART6_RX
```

连接小电脑时需要 TX/RX 交叉并共地。正式视觉串口只传 RMV1 二进制帧，不应混发调试字符串。

## 4. 当前视觉通信功能

### 4.1 已完成

- RMV1 帧头、版本、消息号、长度、flags、seq、session 和发送时间；
- 小端整数与 IEEE754 float32 编解码；
- CRC-16/CCITT-FALSE；
- 串口分包、粘包、前置噪声和候选帧超时恢复；
- `HELLO / HELLO_REPLY`；
- `TIME_SYNC / TIME_REPLY`；
- `HEARTBEAT`，500 ms 超时撤销会话；
- ACK、错误码、重复/冲突/过期操作判断；
- `STATUS`、`DIAGNOSTICS`、`SERVICE_STATE`；
- `STOP_REMOTE`；
- 两颗 IMU 的原始 `POSE`；
- 小Yaw、Pitch、大Yaw的 `JOINT_STATE`。

### 4.2 当前 POSE

| sensor_id | frame_id | 数据来源 | 当前含义 |
|---:|---:|---|---|
| 1 | 1 | `imu_small_pantile` | 小Yaw/Pitch侧IMU原始坐标 |
| 2 | 2 | `imu_big_pantile` | 大Yaw侧IMU原始坐标 |

POSE 包含归一化四元数、角速度、加速度、样本计数器、MCU侧样本时间和坐标系版本。

- 角速度按 `deg/s → rad/s` 转换；
- 加速度按 `g → m/s²` 转换；
- 样本超过 100 ms 不发送；
- 每颗有效 IMU 当前最高发送频率为 50 Hz；
- `sensor_id=0/frame_id=0` 的完整相机支架姿态尚未实现。

CH010 的实际单位、轴序、符号和安装旋转仍需通过实物测试确认。

### 4.3 当前 JOINT_STATE

| axis_id | 关节 | 当前数据来源 |
|---:|---|---|
| 0 | 小Yaw | `ctrl.pantile_motor[YAW]` |
| 1 | Pitch | `DM_motorPitch` |
| 2 | 大Yaw | `DM_motorYaw` |

当前发送角度、速度、可用时的目标角度、在线/使能/有效状态、故障码、温度和反馈年龄。角度单位为 rad，速度单位为 rad/s，当前发送频率为 20 Hz。

没有可靠回零状态时，`homed` 保持为 0。零点、方向、连续角和限位必须通过实车确认。

### 4.4 当前临时发送方式

建立有效 RMV1 会话后：

- 每 20 ms 调用一次 `SendPose()`；
- 每 50 ms 调用一次 `SendJointState()`。

POSE 和 JOINT_STATE 目前尚未接入正式 `SUBSCRIBE` 频率调度；当前固定发送方式仅用于联调。

## 5. 当前安全边界

当前代码保持：

```text
SUPPORTED_CAPS = 0
MODE_REQUEST(VISION) = NOT_READY
```

因此目前：

- 视觉 AIM 可以进入协议格式、时间和序号检查；
- AIM 不会驱动云台电机；
- `RC::OnPC()` 尚未实现；
- 视觉不能控制底盘；
- 视觉不能控制发射；
- 收到 ACK 不代表电机已经执行目标。

## 6. 遥控模式

计划用于视觉自动控制的挡位为：

```text
左开关上 + 右开关上 = AUTOAIM
```

当前 AUTOAIM 仍使用原有遥控器精调逻辑，没有采用 XUC 中的视觉目标。切换到该挡位不会让视觉直接控制车辆。

后续开放视觉控制时，至少需要同时满足：

1. 遥控器在线并位于 AUTOAIM；
2. RMV1 会话与心跳有效；
3. IMU和关节反馈有效；
4. 视觉模式请求成功；
5. AIM目标未过期；
6. 本地限位和故障检查通过；
7. 切出自动挡后立即撤销视觉许可和目标。

## 7. 编译方法

1. 安装 Visual Studio 2019；
2. 安装 VisualGDB 与工程对应的 ARM GCC 工具链；
3. 使用 Visual Studio 打开 `FreeRTOS.sln`；
4. 选择 `Debug|VisualGDB` 或 `Release|VisualGDB`；
5. 执行 Rebuild；
6. 根据本机调试器重新设置下载和调试配置。

工程使用 VisualGDB BSP 中的 STM32F4 HAL、CMSIS 和 FreeRTOS。若脱离 VisualGDB 手工编译，需要自行配置 `TOOLCHAIN_ROOT`、`BSP_ROOT` 等路径。

当前版本已经通过一次 Debug 完整重编译。工程仍存在部分历史警告，主要来自裁判系统、USB HAL和旧代码中的类型用法。

## 8. 克隆后的注意事项

- 不要提交个人 `.user`、`.suo`、IDE缓存和调试器本机路径；
- 首次打开工程时 VisualGDB 可能要求重新选择工具链、BSP和调试器；
- 上电或下载前检查CAN编号、串口实例、波特率、电机零点和限位；
- 未完成实车安全验证前，不要直接开放 `SUPPORTED_CAPS`；
- 不要把原始IMU `1/1`、`2/2` 当成完整相机支架 `0/0` 姿态；
- 联调时先验证通信与反馈，再接入任何运动控制。

## 9. 后续开发顺序

1. 使用串口工具确认CH010单位、轴序和正负号；
2. 验证小Yaw、Pitch、大Yaw的零点、连续角和速度方向；
3. 将POSE和JOINT_STATE接入 `SUBSCRIBE` 调度；
4. 确认两颗IMU和相机的安装变换；
5. 合成 `sensor_id=0/frame_id=0` 的相机支架完整姿态；
6. 增加可信回零、限位和故障状态；
7. 实现 `RC::OnPC()` 和单一明确的视觉控制入口；
8. 验证自动挡许可、遥控抢回、断线和目标超时；
9. 安全测试通过后再逐项开放能力位；
10. 最后再评估视觉发射、底盘和标定功能。

## 10. 相关说明

详细的视觉端消息行为和联调步骤见：

- `给视觉同学的RMV1对接说明.md`
- 上级目录中的 `电控视觉RMV1实施与进度检查.md`

本工程当前用于队内开发和联调，未附带独立开源许可证。
