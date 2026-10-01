# 给视觉同学的 RMV1 对接说明

版本：2026-10-01，原始POSE与JOINT_STATE联调版。  
协议依据：`视觉电控串口协议(1).md`

## 1. 当前交付边界

当前固件已完整编译通过，可以进行RMV1通信、两颗IMU原始POSE和三关节JOINT_STATE联调。

当前只开放反馈，不开放运动控制：

```text
SUPPORTED_CAPS = 0
```

视觉发送HELLO时，`required_caps` 必须设为0。`MODE_REQUEST(VISION)` 当前返回 `NOT_READY`；AIM可以测试格式和拒绝逻辑，但不会驱动云台、底盘或发射机构。

## 2. 串口

- MCU外设：USART6；
- 460800 baud、8数据位、无校验、1停止位、无流控；
- MCU映射：PC6=USART6_TX，PC7=USART6_RX；
- TX/RX交叉连接并共地；
- 实际转接器电平、接头和供电方式接线前由双方确认。

正式串口只传RMV1二进制帧，不混发printf、VOFA、旧SP或裸IMU数据。

## 3. 统一帧格式

所有整数小端，浮点为IEEE754 float32。

| 偏移 | 字段 | 类型 | 要求 |
|---:|---|---|---|
| 0 | magic | byte[4] | ASCII `RMV1` |
| 4 | major | u8 | 1 |
| 5 | minor | u8 | 0 |
| 6 | msg_type | u16 | 见消息表 |
| 8 | payload_len | u16 | 0～256 |
| 10 | flags | u16 | bit0 ACK_REQUEST，bit1 RESPONSE |
| 12 | seq | u32 | 每个方向独立递增 |
| 16 | session_id | u32 | HELLO为0，之后使用回复值 |
| 20 | sender_time_us | u64 | 发送方启动后的单调时间 |
| 28 | payload | byte[N] | 准确长度 |
| 28+N | crc16 | u16 | CCITT-FALSE，小端 |

总长度为 `30+N`。CRC覆盖magic开始的帧头和payload，不包含CRC自身。

CRC参数：poly=0x1021、init=0xFFFF、refin=false、refout=false、xorout=0。测试串 `123456789` 应得到 `0x29B1`。

接收端必须支持分包、粘包、前置噪声和候选帧超时，不能把一次串口read当成一帧。

## 4. 当前消息支持矩阵

| 消息 | 方向 | 当前状态 |
|---|---|---|
| HELLO 0x0001 | PC→MCU | 支持；required_caps必须为0 |
| HELLO_REPLY 0x8001 | MCU→PC | 支持 |
| TIME_SYNC 0x0002 | PC→MCU | 支持 |
| TIME_REPLY 0x8002 | MCU→PC | 支持 |
| HEARTBEAT 0x0003 | PC→MCU | 支持，建议10 Hz |
| SUBSCRIBE 0x0004 | PC→MCU | STATUS/DIAGNOSTICS/SERVICE_STATE支持 |
| ACK 0x00FF | MCU→PC | 支持 |
| STATUS 0x0101 | MCU→PC | 支持 |
| POSE 0x0102 | MCU→PC | 两颗原始IMU已发送；完整支架姿态未发送 |
| JOINT_STATE 0x0103 | MCU→PC | 三轴已发送，待实车校验 |
| SERVICE_STATE 0x0106 | MCU→PC | 当前只报告空闲 |
| DIAGNOSTICS 0x0108 | MCU→PC | 支持 |
| MODE_REQUEST 0x0200 | PC→MCU | SAFE支持；VISION=NOT_READY |
| AIM_SETPOINT 0x0201 | PC→MCU | 校验但不驱动电机 |
| STOP_REMOTE 0x0202 | PC→MCU | 支持，清除远程状态 |
| SERVICE_REQUEST 0x0205 | PC→MCU | UNSUPPORTED |
| 参数消息0x0300～0x0304 | 双向 | UNSUPPORTED |
| 发射/底盘消息 | 双向 | 未开放 |

## 5. 握手、时间同步和心跳

HELLO payload为 `<QIBBH>`，16字节。当前要求nonce为非零随机u64、required_caps=0、min_major=max_major=1、reserved=0、帧头session_id=0。同一请求重发使用相同nonce和payload。

TIME_SYNC payload为 `<Q>`。当前MCU时间字段单位是微秒，但实际来自1 ms tick，不能当成相机曝光硬同步。

HEARTBEAT payload为 `<B3s>`。host_state只允许0待机、1运行、2故障，三个保留字节必须为0。建议10 Hz发送；超过500 ms没有合法心跳，电控结束会话并清除远程目标。

## 6. 当前遥测发送方式

当前订阅建议：STATUS 20 Hz、DIAGNOSTICS 1 Hz、SERVICE_STATE 1 Hz。

POSE和JOINT_STATE目前处于临时联调阶段：

- 每颗原始IMU的POSE：50 Hz；
- JOINT_STATE：20 Hz；
- 建立会话后自动发送；
- 暂不由SUBSCRIBE控制频率。

因此建立会话后应直接准备接收 `0x0102` 和 `0x0103`。正式版本会再改为协议规定的SUBSCRIBE调度。

## 7. POSE 0x0102

每帧payload固定68字节。当前会收到：

| sensor_id | frame_id | 含义 |
|---:|---:|---|
| 1 | 1 | 小Yaw/Pitch侧IMU原始坐标 |
| 2 | 2 | 大Yaw侧IMU原始坐标 |

当前不会发送 `sensor_id=0/frame_id=0`，因为相机支架完整姿态尚未完成安装旋转和Pitch运动学合成。

数据约定：

- 四元数顺序w、x、y、z，发送前归一化；
- 角速度单位rad/s；
- 加速度单位m/s²；
- `valid_flags=0x0027`；
- `source_counter` 只在真实新样本出现时增加；
- 样本超过100 ms不发送；
- MCU样本时间来自接收tick；
- IMU设备时间暂填0且未置有效位。

当前按CH010字段 `pitch/roll/yaw` 分别对应传感器X/Y/Z发送。这个轴映射、原始单位和符号仍需实物验证，因此1/1和2/2只能按原始传感器坐标使用。

## 8. JOINT_STATE 0x0103

payload固定112字节，一帧包含3个轴：

| axis_id | 关节 | 当前约定 |
|---:|---|---|
| 0 | 小Yaw | 角度rad、速度rad/s，使用当前机械前向编码零点 |
| 1 | Pitch | 角度rad、速度rad/s，当前按抬头为正处理 |
| 2 | 大Yaw | 角度rad、速度rad/s，使用机械前向零点并连续解环 |

当前按已有反馈填写在线、使能、角度、速度、采用目标、温度、故障和反馈年龄。没有可靠来源的字段保持0并清除有效位。

注意：`homed` 当前始终为0；DM温度和驱动输出比例暂无可靠来源；三轴零点、方向、连续角和在线超时仍需实车确认。不能因为角度为0就认为已经回零。

## 9. STATUS和AIM当前行为

STATUS中的轴掩码、回零、视觉许可、发射许可和 `last_applied_aim_seq` 等字段目前仍可能为0，这表示能力尚未开放。

AIM的长度、frame_epoch、坐标系、有效位、float、时间、deadline和seq会被检查。但VISION模式不会成功，所以AIM不会进入电机控制。ACK=OK也只表示协议请求被接受，不代表电机已执行。

## 10. 遥控挡位

计划视觉自动挡为：

```text
左开关上 + 右开关上 = AUTOAIM
```

当前AUTOAIM仍使用遥控摇杆精调，不采用XUC视觉目标。当前任何挡位都不会让视觉直接控制车辆，也没有开放视觉发射。

## 11. 当前联调步骤

1. HELLO建立会话并确认session_id；
2. 以10 Hz发送HEARTBEAT；
3. 订阅STATUS、DIAGNOSTICS和SERVICE_STATE；
4. 接收两个 `0x0102` 原始POSE流；
5. 接收一个 `0x0103` 三关节状态流；
6. 检查四元数模长、source_counter和时间单调；
7. 检查静置加速度、单轴转动时角速度轴和符号；
8. 检查三个关节角度、速度、零点和正方向；
9. 停止心跳，确认约500 ms后会话结束且遥测停止；
10. 确认VISION返回NOT_READY且AIM不产生运动。

完成以上步骤只表示通信和反馈链路通过。相机支架完整姿态、SUBSCRIBE调度、视觉控制、遥控抢回、限位和断线安全仍需后续单独验收。
