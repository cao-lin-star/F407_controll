# UART4 ASCII 日志与命令使用说明

> 适用工程：`G:\work\ROS2\Chassis`  
> 固件接口：STM32F407 UART4  
> 默认参数：115200、8N1、无流控、3.3 V TTL  
> 更新日期：2026-08-16

## 1. UART4 与 USART1 的区别

| 接口 | 引脚 | 格式 | 用途 |
|---|---|---|---|
| USART1 | PA9 TX、PA10 RX | `AA 55` 二进制协议 | RK3576 正式通信；普通串口工具显示为乱码属正常现象 |
| UART4 | PC10 TX、PC11 RX | 可读 ASCII 文本 | 电脑台架调试、查看全部状态、发送速度和 PID 命令 |

当前已实测的 COM19 接在 USART1，因此不能向当前接法的 COM19 发送 UART4 ASCII 命令。要使用本文命令，应将 DAP/USB-TTL 串口连接为：

- PC10/UART4_TX → DAP 或 USB-TTL 的 RX。
- PC11/UART4_RX ← DAP 或 USB-TTL 的 TX。
- F407 GND ↔ 串口模块 GND。
- 串口必须是 3.3 V TTL；5 V TX 不得接入 PC11。
- PC10/PC11 与板载 SD 卡 DAT2/DAT3 复用，调试时必须拔出 SD 卡并关闭 SDIO。

如果只移动同一个 DAP 虚拟串口的 TX/RX 线，Windows COM 号通常不变；如果增加第二个 USB-TTL，则会出现新的 COM 号。以下示例假设 UART4 为 COM20，使用时替换为实际编号。

## 2. 串口工具设置

| 项目 | 设置 |
|---|---|
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | None/无 |
| 流控 | None/无 |
| 接收显示 | ASCII/文本，关闭十六进制显示 |
| 发送格式 | ASCII |
| 行尾 | CRLF 推荐，LF 也支持 |
| 自动发送间隔 | 运动命令如需连续刷新，设置 50 ms；更推荐使用项目脚本 |

命令区分大小写，必须使用小写。数字支持 `0.05`、`-0.02`、`+0.1`，不支持 `1e-2` 这种科学计数法。

## 3. UART4 会打印哪些日志

UART4 当前有六类文本行：

| 行首 | 何时出现 | 含义 |
|---|---|---|
| `READY` | MCU 启动/复位时一次 | UART4 调试服务已经启动 |
| `TEL` | 默认每 200 ms 一次，或执行 `status` | 全部实时遥测 |
| `PARAM` | 执行 `params` 或成功修改 PID 后 | 当前控制参数 |
| `PS2` | 执行 `ps2` | 手柄模式、摇杆、控制权和通信计数 |
| `HELP` / `NOTE` | 执行 `help` | 命令列表和看门狗提示 |
| `OK` | 命令合法并已执行 | 成功响应 |
| `ERR` | 命令或参数无效 | 错误响应及正确用法 |

### 3.1 READY 启动行

MCU 启动时输出：

```text
READY,Footbath Chassis ASCII debug UART4 115200 8N1; type help
```

如果 MCU 早已启动、之后才打开电脑串口，可能看不到 READY；只要仍能看到 TEL 或执行 `help` 得到响应，UART4 就在工作。

### 3.2 TEL 实时遥测行

默认每 200 ms，也就是 5 Hz 输出一行。格式如下：

```text
TEL,t_ms=123456,tgt_l_mm_s=0,tgt_r_mm_s=0,spd_l_mm_s=0,spd_r_mm_s=0,pwm_l_x100=0,pwm_r_x100=0,enc_l=0,enc_r=0,odom_v_mm_s=0,odom_w_mrad_s=0,tof_l_mm=26,tof_r_mm=34,us_mm=345,ax_mg=3,ay_mg=-8,az_mg=1001,gx_mrad_s=2,gy_mrad_s=-1,gz_mrad_s=0,open_loop=0,source=0,ps2_link=1,ps2_mode=0x73,ps2_manual=0,ps2_r1=0,ps2_stop=0,ps2_rx=128,ps2_ly=128,fault=0x0000,valid=0x000F,obstacle=0x0000,sensor_fault=0x0000
```

上面是格式示例，IMU 数字不是当前设备的实测记录。2026-08-16 通过 USART1 等价遥测实测到的静止距离约为：左 ToF 25～27 mm、右 ToF 33～35 mm、超声波 345 mm。

#### TEL 字段说明

| 字段 | 单位/换算 | 含义 |
|---|---|---|
| `t_ms` | ms | MCU 上电运行时间；除以 1000 得到秒 |
| `tgt_l_mm_s` | mm/s | 左轮目标速度；正数为配置定义的前进方向 |
| `tgt_r_mm_s` | mm/s | 右轮目标速度 |
| `spd_l_mm_s` | mm/s | 左编码器计算的实测速度 |
| `spd_r_mm_s` | mm/s | 右编码器计算的实测速度 |
| `pwm_l_x100` | 百分比×100 | 左轮输出；`1234`=12.34%，`-800`=-8.00% |
| `pwm_r_x100` | 百分比×100 | 右轮输出；当前绝对值最大 3500，即 35.00% |
| `enc_l` | tick | 左编码器累计计数；当前理论每轮一圈约 3468 tick |
| `enc_r` | tick | 右编码器累计计数 |
| `odom_v_mm_s` | mm/s | 编码器里程计计算的车体线速度 |
| `odom_w_mrad_s` | mrad/s | 车体角速度；除以 1000 得 rad/s |
| `tof_l_mm` | mm | 左 TOFSense-M S 距离 |
| `tof_r_mm` | mm | 右 TOFSense-M S 距离 |
| `us_mm` | mm | 超声波距离 |
| `ax_mg` / `ay_mg` / `az_mg` | mg | MPU6050 三轴加速度；1000 mg 约等于 1 g |
| `gx_mrad_s` / `gy_mrad_s` / `gz_mrad_s` | mrad/s | MPU6050 三轴角速度；除以 1000 得 rad/s |
| `open_loop` | 0/1 | `1` 表示 UART4 开环 PWM 调试模式，两个 PID 均已旁路 |
| `source` | 枚举 | 0=无控制源，1=RK，2=UART4调试，3=PS2；固定优先级3>2>1 |
| `ps2_link` / `ps2_mode` | 状态/十六进制 | PS2链路有效状态和模式ID，模拟红灯通常为0x73 |
| `ps2_manual` / `ps2_r1` / `ps2_stop` | 0/1 | 手动控制权、R1行驶使能和锁存停车 |
| `ps2_rx` / `ps2_ly` | 0～255 | 右摇杆左右与左摇杆上下原始值 |
| `fault` | 十六进制位图 | 底盘控制/通信故障 |
| `valid` | 十六进制位图 | 哪些传感器的当前数值有效 |
| `obstacle` | 十六进制位图 | 台阶、近障和红外输入状态 |
| `sensor_fault` | 十六进制位图 | 具体传感器故障/超时状态 |

`tgt` 是期望值，`spd` 是编码器实测值，`pwm` 是 PID 输出。闭环正常时，`spd` 应跟随 `tgt`，方向符号一致，PWM 不应长时间顶到 ±3500。

### 3.3 PARAM 参数行

执行 `params` 后，默认输出类似：

```text
PARAM,pid_l_kp_x10000=500,pid_l_ki_x10000=10,pid_l_kd_x10000=0,pid_r_kp_x10000=500,pid_r_ki_x10000=10,pid_r_kd_x10000=0,wheel_track_mm=330,max_linear_mm_s=1000,max_angular_mrad_s=1500,cmd_timeout_ms=500
```

| 字段 | 换算后的当前值 |
|---|---:|
| `pid_l_kp_x10000=500` | 左 Kp = 0.05 |
| `pid_l_ki_x10000=10` | 左 Ki = 0.001 |
| `pid_l_kd_x10000=0` | 左 Kd = 0 |
| 右轮三个 PID 字段 | 右 Kp/Ki/Kd，换算方法相同 |
| `wheel_track_mm=330` | 轮距 0.330 m |
| `max_linear_mm_s=1000` | 最大车体线速度绝对软件上限 1.00 m/s |
| `max_angular_mrad_s=1500` | 最大车体角速度 1.50 rad/s |
| `cmd_timeout_ms=500` | 500 ms 收不到新速度命令就停机 |

## 4. 状态位如何解读

### 4.1 `fault`

| 位值 | 含义 | 常见处理 |
|---:|---|---|
| `0x0001` | 500 ms 命令超时 | 上电未收到命令或 STOP 后出现属正常；发送合法零速度可清除 |
| `0x0002` | UART 接收错误/溢出 | 检查电平、共地、波特率和 EMI |
| `0x0004` | USART1 二进制 CRC 错误 | 检查正式通信线和干扰 |
| `0x0008` | USART1 协议格式错误 | 不要向 USART1 发送 ASCII；复位清除保留故障 |
| `0x0010` | 左编码器跳变/停转 | 锁存停机；修复后按 NRST |
| `0x0020` | 右编码器跳变/停转 | 锁存停机；修复后按 NRST |
| `0x0040` | 发送队列溢出 | 检查串口负载/主循环 |
| `0x0080` | 控制周期超时 | 停机并检查阻塞或中断负载 |
| `0x0100` | 非法命令 | 检查数值、范围和编码器锁存 |
| `0x0200` | 急停 PG4 有效 | PWM/EN 清零；释放后按安全流程人工复位 |
| `0x0400` | 障碍联锁停机 | 当前默认尚未启用传感器自动硬停 |
| `0x0800` | 安全传感器配置错误 | 检查联锁与驱动配置 |

多个故障会相加。例如 `fault=0x0130` 表示 `0x0100 + 0x0020 + 0x0010`，即非法命令加左右编码器故障。

执行 `stop` 后，控制命令被置为无效，随后看到 `fault=0x0001` 是当前固件的正常看门狗状态，不表示 STOP 失败；关键是目标和 PWM 必须为 0。

### 4.2 `valid`

| 位值 | 有效数据 |
|---:|---|
| `0x0001` | 左 ToF |
| `0x0002` | 右 ToF |
| `0x0004` | 超声波 |
| `0x0008` | MPU6050 |
| `0x000F` | 四类数据全部有效 |

必须先检查有效位，再使用距离或 IMU 数字。例如：

```text
valid=0x000E,sensor_fault=0x0008
```

表示右 ToF、超声波、IMU 有效，但左 ToF 当前无效；此时不能使用 `tof_l_mm` 做安全判断。

### 4.3 `obstacle`

| 位值 | 含义 |
|---:|---|
| `0x0001` | 左 ToF 检测到台阶 |
| `0x0002` | 右 ToF 检测到台阶 |
| `0x0004` | 超声波近障 |
| `0x0008` | 左红外 PG2 低电平触发 |
| `0x0010` | 右红外 PG3 低电平触发 |

当前`FRONT_OBSTACLE_SAFETY_ENABLE=1`、`CLIFF_SAFETY_ENABLE=1`：超声`0x0004`和左右ToF台阶`0x0001/0x0002`进入F407方向性停车掩码；只阻止含前向轮速的速度/PWM。触发后必须先发`stop`或零速，再发新的低速倒退命令；红外仍关闭。ToF无效超过250 ms按台阶处理；TEL新增`tof_l_rxerr/tof_r_rxerr`累计接收/校验/格式错误。超声无回波/超量程上报`us_mm=4000, valid bit2=1`，表示量程内清空，不替代实体急停。

### 4.4 `sensor_fault`

| 位值 | 含义 |
|---:|---|
| `0x0001` | MPU6050/I2C 故障 |
| `0x0002` | ToF 协议模式未确认 |
| `0x0004` | 超声波协议模式未确认 |
| `0x0008` | 左 ToF 超时/帧错误 |
| `0x0010` | 右 ToF 超时/帧错误 |
| `0x0020` | 超声波异常短脉冲或 ECHO 上升后卡高 |

## 5. UART4 ASCII 命令完整列表

### 5.1 `help`

发送：

```text
help
```

返回：

```text
HELP,status | params | ps2 | stream on|off | stop | vel <linear_mps> <angular_rps> | wheel <left_mps> <right_mps> | pwm <left_percent> <right_percent> | pid <left|right|both> <kp> <ki> <kd>
NOTE,vel/wheel/pwm is one watchdog pulse; repeat at 20Hz. pwm bypasses PID, is UART4-only, and is limited to +/-35 percent. PID changes stop the motors and are RAM-only.
```

### 5.2 `status`

发送：

```text
status
```

立即输出一行 `TEL`，即使已经执行 `stream off` 也可使用。

### 5.3 `params`

发送：

```text
params
```

返回一行 `PARAM`，用于确认当前 PID、轮距、限速和看门狗。

### 5.3a `ps2`

发送`ps2`后返回一行手柄状态，包括连接、模式、手动控制权、R1、停车锁存、四轴原始值、换算速度及有效/无效帧计数。完整接线和控制规则见[`PS2无线手柄接线与控制说明.md`](PS2无线手柄接线与控制说明.md)。

### 5.4 `stream on` / `stream off`

```text
stream off
```

返回：

```text
OK,stream=off
```

停止 5 Hz 自动 TEL，但 `status` 仍可单次读取。

```text
stream on
```

返回 `OK,stream=on` 并立即恢复 5 Hz TEL。

### 5.5 `stop`

```text
stop
```

返回：

```text
OK,stopped
```

作用：目标速度清零、左右 PID 清零、PWM=0、IBT-2 EN=0，并让当前命令失效。这是调试结束时必须发送的命令。

### 5.6 `vel <linear_mps> <angular_rps>`

设置车体线速度和角速度：

```text
vel 0.05 0
```

表示直行 0.05 m/s、角速度 0 rad/s。成功返回：

```text
OK,vel accepted; watchdog=500ms
```

其他示例：

```text
vel -0.02 0
vel 0 0.20
vel 0.05 -0.20
vel 0 0
```

当前车体绝对软件限值为线速度 ±1.00 m/s、角速度 ±1.50 rad/s。`vel` 超限时固件会钳位到限值；1.00 m/s 不是推荐调试速度，首次有载调试仍应从 0.05 m/s 逐档增加。

每条 `vel` 只刷新一次 500 ms 看门狗。只在串口工具中手工发送一次时，电机最多运动约 500 ms；要连续运行必须每 50 ms 左右重发。

### 5.7 `wheel <left_mps> <right_mps>`

直接设置左右轮闭环速度：

```text
wheel 0.02 0
```

只让左轮以 0.02 m/s 正转，右轮停止。

```text
wheel 0 0.02
wheel 0.02 0.02
wheel -0.02 -0.02
wheel 0.02 -0.02
```

分别表示右轮单独转、双轮前进、双轮后退、原地旋转。每轮允许范围为 ±1.00 m/s，超过时返回：

```text
ERR,wheel speed exceeds configured limit
```

`wheel` 与 `vel` 一样需要约 20 Hz 重发，并受 500 ms 看门狗保护。

### 5.8 `pwm <left_percent> <right_percent>`

严格开环调试命令，例如：

```text
pwm 12 0
pwm 0 -18
```

`pwm` 直接指定逻辑左右轮占空比百分数，两个 PID 均被旁路。允许范围为 ±35%，只在 UART4 调试口开放，USART1/RK3576 协议不支持该命令。成功返回：

```text
OK,pwm accepted; PID bypassed; watchdog=500ms
```

必须约 20 Hz 重发并在结尾发送 `stop`。遥测 `open_loop=1` 表示开环模式有效；停止发送超过 500 ms 后 PWM 自动清零并退出该模式。该命令仅用于轮子置空、底盘固定条件下测量死区，禁止作为正常行驶控制接口。

### 5.9 `pid <left|right|both> <kp> <ki> <kd>`

修改左右 PID：

```text
pid both 0.05 0.001 0
```

只改左轮或右轮：

```text
pid left 0.055 0.001 0
pid right 0.055 0.001 0
```

成功后返回：

```text
OK,pid updated in RAM; motors stopped
PARAM,...
```

规则：

- 修改 PID 前固件会先 STOP。
- Kp、Ki、Kd 各自只能为 0～10。
- 参数只保存在 RAM；按 NRST 或重新上电后恢复 `include/board_config.h` 默认值。
- 初调时 Kd 保持 0，每次只改一个参数并以约 10%～20% 的幅度调整。

## 6. 常见 ERR 响应

| 返回 | 含义 |
|---|---|
| `ERR,unknown command; type help` | 命令拼错、使用大写或发送了不支持的命令 |
| `ERR,usage: stream on|off` | stream 参数不正确 |
| `ERR,usage: vel <linear_mps> <angular_rps>` | 缺参数、多参数或数字格式不支持 |
| `ERR,vel/wheel/pwm rejected; higher-priority source or safety recovery active` | PS2正在占权，或底盘仍处于安全故障恢复等待；查看`source`、`ps2_*`和`fault` |
| `ERR,usage: wheel <left_mps> <right_mps>` | 左右轮参数不完整 |
| `ERR,wheel speed exceeds configured limit` | 任一轮绝对速度大于 1.00 m/s |
| `ERR,usage: pid ...; gains 0..10` | PID 侧别/数量/范围错误 |

## 7. 使用项目脚本

项目脚本比普通串口工具更适合连续运动，因为会自动按 20 Hz 刷新命令并在结束时发送 STOP。

### 7.1 显示帮助、参数和一行状态

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --baud 115200 `
  --command help --command params --command status --duration 3
```

### 7.2 持续监视

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --baud 115200 --monitor
```

按 Ctrl+C 退出。

### 7.3 发送单条命令

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --command "stream off" --command status --duration 2
```

### 7.4 安全的短时双轮命令

首次必须保持 24 V 关闭；实际电机测试必须轮子悬空、急停可用：

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --linear 0.02 --angular 0 --duration 0.5
```

脚本以约 20 Hz 重发 `vel 0.02 0`，0.5 秒后自动发送 `stop`。

### 7.5 左右轮分别测试

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --left 0.02 --right 0 --duration 0.5

& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM20 --left 0 --right 0.02 --duration 0.5
```

## 8. 推荐首次使用顺序

1. 24 V 保持关闭，连接 PC10、PC11 和 GND，拔出 SD 卡。
2. 打开 115200 ASCII 串口；按 NRST，确认 READY 和 TEL。
3. 依次发送 `help`、`params`、`ps2`、`status`。
4. 确认静止时 `tgt=0`、`spd=0`、`pwm=0`、编码器不跳变。
5. 检查 `valid` 和 `sensor_fault`，健康目标为 `0x000F/0x0000`。
6. 手转左右轮，检查对应 `enc` 变化和前进为正。
7. 发送 `stream off`、`status`、`stream on`，确认日志控制正常。
8. 发送 `vel 0 0`，确认返回 OK；等待超过 500 ms 后允许看到超时位，但 PWM 必须保持 0。
9. 只有全部静态项目通过后，轮子悬空并打开 24 V，用项目脚本测试 0.02 m/s、0.5 秒。
10. 每次运动后确认 `tgt=0`、`pwm=0`、EN=0；出现 `0x0010/0x0020/0x0200` 时停止测试并复位排查。

UART4 是台架调试接口，USART1 仍可同时留给 RK3576。但调试运动时必须暂停 RK 的速度发布器，因为 UART4 和 USART1 的最后一条合法速度命令都会刷新同一个底盘控制目标。
