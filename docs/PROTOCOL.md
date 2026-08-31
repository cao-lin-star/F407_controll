# RK3576 ↔ STM32F407 底盘二进制协议 v1

## 1. 物理层

- USART1（PA9/PA10）有线 UART，115200 8N1，无硬件流控。
- USART1 只承载本协议二进制帧；普通终端显示乱码是正常现象，严禁混入 ASCII 日志。
- 默认 3.3 V TTL，TX/RX 交叉并共地；不是 RS-232。
- 所有多字节整数、CRC 和 IEEE-754 float32 均为小端。
- 禁止发送 NaN/Inf 速度或传感器数据。

## 2. 帧格式

| 偏移 | 字段 | 长度 | 说明 |
|---:|---|---:|---|
| 0 | SOF1 | 1 | `0xAA` |
| 1 | SOF2 | 1 | `0x55` |
| 2 | version | 1 | `0x01` |
| 3 | msg_type | 1 | 见消息表 |
| 4 | seq | 2 | `uint16`，发送端独立递增 |
| 6 | payload_len | 2 | `uint16`，最大 64 |
| 8 | payload | N | 紧密排列，无填充 |
| 8+N | crc16 | 2 | 从 version 到 payload 末尾 |

CRC 为 CRC16-CCITT-FALSE：`poly=0x1021`、`init=0xFFFF`、`refin=false`、`refout=false`、`xorout=0x0000`。标准串 `123456789` 的结果为 `0x29B1`。

## 3. 消息

| 类型 | 名称 | 方向 | 长度 | payload |
|---:|---|---|---:|---|
| `0x01` | CMD_VEL | RK→F407 | 8 | `float32 linear_mps, angular_rps` |
| `0x02` | ODOM | F407→RK | 28 | `float32 x_m,y_m,yaw_rad,linear_mps,angular_rps,int32 left_ticks,right_ticks` |
| `0x03` | HEARTBEAT | F407→RK | 6 | `uint32 uptime_ms,uint16 fault_flags` |
| `0x04` | STOP | RK→F407 | 0 | 空 |
| `0x05` | RANGE_STATUS | F407→RK | 16 | 见下节 |
| `0x06` | IMU_RAW | F407→RK | 28 | 见下节 |
| `0x7F` | DEBUG_STATUS | F407→PC/RK | 64 | 可选诊断帧，见下节 |

### RANGE_STATUS

```text
float32 tof_left_m
float32 tof_right_m
float32 ultrasonic_m
uint16 valid_flags
uint16 packed_status
```

`valid_flags`：bit0 左 ToF、bit1 右 ToF、bit2 超声波、bit3 IMU。`packed_status` 的低 8 位是 obstacle flags，高 8 位是 sensor fault flags。无效距离必须结合 valid 位判断，不能把数值 0 自动解释为近障碍。超声波无回波或超过 4.0 m 时上报有限值 `4.0 m` 且 valid=1，表示量程内清空；只有异常短脉冲或 ECHO 上升后持续不下降才清 valid 并置超声故障。

### IMU_RAW

```text
uint32 stamp_ms
float32 accel_x_mps2, accel_y_mps2, accel_z_mps2
float32 gyro_x_rps, gyro_y_rps, gyro_z_rps
```

F407 端 MPU6050 配置为 ±4 g、±500 °/s，上传前转换为 SI 单位。

### DEBUG_STATUS

`0x7F` 是不改变 v1 控制语义的可选诊断消息，当前以 5 Hz 发送。RK 正式驱动可以解析，也可以安全忽略未知类型；台架脚本 `tools/closed_loop_test.py` 会验 CRC 后输出 CSV。

```text
uint32  uptime_ms
float32 target_left_mmps, target_right_mmps
float32 measured_left_mmps, measured_right_mmps
float32 output_left_percent, output_right_percent
int32   left_ticks, right_ticks
float32 odom_linear_mps, odom_angular_rps
float32 tof_left_m, tof_right_m, ultrasonic_m
uint16  fault_flags, valid_flags, obstacle_flags, sensor_fault_flags
```

字段按上列紧密序列化，总长 64 字节，不依赖 C 结构体填充。正式串口严禁混入裸 `printf` 文本。

## 4. 安全规则

- F407 只执行帧头、版本、长度、CRC 和 payload 长度全部正确的命令。
- `CMD_VEL` 必须恰好 8 字节，`STOP` 必须为 0 字节。
- 连续 500 ms 没有有效 `CMD_VEL` 时，F407 清 PID 并强制停机。
- 错 CRC、未知类型、错误长度不能刷新命令看门狗。
- STOP 校验通过后立即停机；恢复必须重新发送有效 CMD_VEL。
- RK 正常运行时建议以 20–50 Hz 连续发送 CMD_VEL，退出时发送 STOP。
- RK 侧心跳超时或收到非零 MCU fault 可额外发送 STOP，但不能替代 F407 独立 500 ms 保护。
- F407运动来源统一按PS2 > UART4 > RK仲裁；高优先级来源租约有效时，低优先级CMD_VEL可解析计数但不更新目标。
- PS2仅在手动模式且R1按住时占权；R1松开立即停车并释权，避免手柄关机后接收器仍返回伪合法中位帧而永久阻塞UART4/RK。
- PS2连续100 ms无合法帧时立即停车、清除手动模式并释权；外部STOP会取消PS2手动使能；UART4租约为500 ms，停止刷新后RK后续新命令可接管。
- R1松开、SELECT、RK正式STOP、UART4 `stop`、急停和障碍联锁均立即停车，不等待正常速度斜坡。
- 前障碍/台阶联锁必须在传感器实测和基线标定后才启用；未标定时驱动与诊断可工作，但不参与硬停决策。

### 4.1 可恢复故障规则

- 所有运动相关故障仍然先立即清 PWM、关闭电机输出并使当前命令失效；解除故障只恢复“可接收新命令”状态，不会重新执行故障前的非零速度。
- 左/右编码器跳变或堵转：收到有效零速度或 `STOP` 后，左右编码器每个 10 ms 周期的增量绝对值都不大于 2 count，并连续稳定 500 ms，才清除编码器故障。随后必须再发送一条新的非零命令。
- 急停和障碍：物理/传感器条件必须先消失，再收到零速度或 `STOP`；条件仍有效时每次控制循环都会重新保持停机。
- 控制周期严重超期：立即停机；连续 1 s 未再次超期且已完成零速/`STOP` 确认后恢复。
- 非有限或恢复等待期间的非零命令：立即停机；合法零速度或 `STOP` 可确认恢复。
- UART RX、CRC、协议格式和 TX 队列溢出是瞬态诊断位；连续 1 s 未再次出现同类错误后自动清除。它们不会刷新 500 ms 运动看门狗。
- `FAULT_CMD_TIMEOUT` 在收到新的合法命令后清除；旧命令已失效，不存在超时后自动续驶。
- FreeRTOS 栈溢出、断言和 Cortex 硬件异常不属于 `fault_flags` 的运行时恢复范围，仍保持硬停并需要排查后复位。


## 5. fault_flags

| 位 | 掩码 | 含义 |
|---:|---:|---|
| 0 | `0x0001` | CMD_VEL 超时/尚无命令 |
| 1 | `0x0002` | UART RX 错误或环形缓冲溢出 |
| 2 | `0x0004` | CRC 错误 |
| 3 | `0x0008` | 版本、长度、类型或 payload 格式错误 |
| 4 | `0x0010` | 左编码器跳变/无反馈，立即停机，可按受控条件恢复 |
| 5 | `0x0020` | 右编码器跳变/无反馈，立即停机，可按受控条件恢复 |
| 6 | `0x0040` | UART TX 队列溢出 |
| 7 | `0x0080` | 10 ms 控制调度严重超期 |
| 8 | `0x0100` | 非有限命令，或恢复等待期间收到非零命令 |
| 9 | `0x0200` | 外部急停输入有效 |
| 10 | `0x0400` | 障碍/台阶硬停触发 |
| 11 | `0x0800` | 启用了安全联锁但传感器配置不完整 |

## 6. ROS2 映射

| 协议 | ROS2 |
|---|---|
| CMD_VEL | 订阅 `/cmd_vel` |
| ODOM | 发布 `/odom` 和 `odom→base_footprint` |
| HEARTBEAT/fault | 发布 `/diagnostics` |
| RANGE_STATUS | 发布 `/range/tof_left`、`/range/tof_right`、`/range/ultrasonic` |
| IMU_RAW | 发布 `/imu/data_raw` |
| DEBUG_STATUS | 可选调试/记录，不是 Nav2 必需接口 |

两端若修改 `0x01`–`0x06` 的正式消息，必须同步更新。STM32 定义位于 `include/protocol.h`，RK 定义位于 `rk3576_footbath_base/include/.../protocol.hpp`；`0x7F` 可由 RK 选择忽略。

## 7. 独立 ASCII 调试口

F407 另用 UART4（PC10/PC11，115200 8N1）提供可读 ASCII 遥测和台架命令。该接口不修改 USART1 帧格式，RK3576 端无需改变；UART4 的接线、字段、命令和十六进制解码表见 [`SERIAL_DEBUG_GUIDE.md`](SERIAL_DEBUG_GUIDE.md)。

## 8. PS2本地手动控制

PS2接收器使用PA4 DATA、PA5 CMD、PA6 ATT/CS、PA7 CLK和3.3V软件时序。它不修改USART1消息格式；RK3576无需新增协议字段。手动模式通过START切换，R1为持续行驶使能，SELECT为锁存停车，左摇杆上下生成`linear_mps`，右摇杆左右生成`angular_rps`。详细接线和验收见[`PS2无线手柄接线与控制说明.md`](PS2无线手柄接线与控制说明.md)。
