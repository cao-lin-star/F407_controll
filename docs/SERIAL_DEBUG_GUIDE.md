# STM32F407 底盘双串口调试与协议说明

## 1. 当前接口分工

| 用途 | F407 引脚 | 连接方式 | 参数 | 数据格式 |
|---|---|---|---|---|
| RK3576 正式通信 | PA9 / USART1_TX → RK RX；PA10 / USART1_RX ← RK TX；GND 共地 | 3.3 V TTL，TX/RX 交叉 | 115200、8N1、无流控 | `AA 55` 二进制协议 v1 |
| DAP/电脑调试 | PC10 / UART4_TX → DAP RX；PC11 / UART4_RX ← DAP TX；GND 共地 | 3.3 V TTL，TX/RX 交叉 | 115200、8N1、无流控 | 可读 ASCII |
| 超声波 | PC0 → Trig；PB6 / TIM4_CH1 ← Echo | Echo 必须不高于 3.3 V | 1 MHz 输入捕获 | 触发/回波脉宽 |

UART4 的 PC10/PC11 与霸天虎板载 SD 卡 DAT2/DAT3 复用。使用调试串口时不要插 SD 卡，也不要同时启用 SDIO。DAP 只连接 UART4，USART1 可一直留给 RK3576。

如果超声波模块以 5 V 供电，Echo 不能直接送入 PB6，必须使用电阻分压或电平转换，将高电平限制在 3.3 V；Trig、Echo 和 F407 必须共地。

## 2. 为什么 USART1 看起来是乱码

USART1 不是文本日志口，而是固定格式的二进制链路。串口终端按 ASCII 显示 `AA 55 01 7F ...` 中的任意字节时，会出现方框、问号或乱码，这是正常现象，不是波特率错误。查看 USART1 时应使用十六进制显示，或运行 `tools/closed_loop_test.py` 让脚本完成拆包和 CRC 校验。

不要向 USART1 发送普通文本，例如 `vel 0.1 0`；F407 会把它视为协议噪声。正式运动命令仍为二进制 `CMD_VEL`。

## 3. USART1 十六进制帧含义

每帧均为小端：

| 帧内偏移 | 长度 | 含义 |
|---:|---:|---|
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 协议版本 `01` |
| 3 | 1 | 消息类型 |
| 4 | 2 | `seq`，发送端递增 |
| 6 | 2 | payload 长度 N |
| 8 | N | payload |
| 8+N | 2 | CRC16-CCITT-FALSE，小端；计算范围为版本至 payload 末尾 |

消息类型：

| 类型 | 方向 | 含义 |
|---:|---|---|
| `01` | RK → F407 | `CMD_VEL`：float32 线速度 m/s、float32 角速度 rad/s |
| `02` | F407 → RK | 里程计、车体速度、左右编码器累计值 |
| `03` | F407 → RK | 运行时间和故障位 |
| `04` | RK → F407 | 立即停止，无 payload |
| `05` | F407 → RK | 左右 ToF、超声波、有效位和传感器状态 |
| `06` | F407 → RK | MPU6050 三轴加速度和三轴角速度 |
| `7F` | F407 → RK/PC | 5 Hz 综合调试状态；RK 可忽略 |

`7F DEBUG_STATUS` 的 payload 总长为 64 字节：

| payload 偏移 | 类型 | 含义 |
|---:|---|---|
| 0 | uint32 | `uptime_ms` |
| 4 / 8 | float32 | 左/右目标速度，mm/s |
| 12 / 16 | float32 | 左/右实测速度，mm/s |
| 20 / 24 | float32 | 左/右输出占空比，% |
| 28 / 32 | int32 | 左/右编码器累计 tick |
| 36 / 40 | float32 | 里程计线速度 m/s、角速度 rad/s |
| 44 / 48 / 52 | float32 | 左 ToF、右 ToF、超声波，m |
| 56 / 58 / 60 / 62 | uint16 | `fault`、`valid`、`obstacle`、`sensor_fault` |

`valid` 位：bit0 左 ToF、bit1 右 ToF、bit2 超声波、bit3 IMU。只有对应位为 1 时距离或 IMU 数值才有效。

`obstacle` 位：bit0 左侧台阶、bit1 右侧台阶、bit2 超声波近障、bit3 左红外、bit4 右红外。

`sensor_fault` 位：bit0 MPU6050、bit1 ToF 协议未确认、bit2 超声波协议未确认、bit3 左 ToF、bit4 右 ToF、bit5 超声波。

完整的 `fault` 位定义和 CRC 规则见 `docs/PROTOCOL.md`。

## 4. UART4 ASCII 遥测

上电后调试口先输出：

```text
READY,Footbath Chassis ASCII debug UART4 115200 8N1; type help
```

默认每 200 ms 输出一行 `TEL`。主要字段如下：

| 字段 | 含义/换算 |
|---|---|
| `t_ms` | F407 上电时间，ms |
| `tgt_l_mm_s`、`tgt_r_mm_s` | 左右轮目标速度，mm/s |
| `spd_l_mm_s`、`spd_r_mm_s` | 左右轮编码器实测速度，mm/s |
| `pwm_l_x100`、`pwm_r_x100` | 占空比百分数乘 100；例如 `1234` = 12.34% |
| `enc_l`、`enc_r` | 左右编码器累计 tick |
| `odom_v_mm_s` | 车体里程计线速度，mm/s |
| `odom_w_mrad_s` | 车体里程计角速度，mrad/s |
| `tof_l_mm`、`tof_r_mm`、`us_mm` | 左 ToF、右 ToF、超声波距离，mm |
| `ax_mg`、`ay_mg`、`az_mg` | 加速度，mg |
| `gx_mrad_s`、`gy_mrad_s`、`gz_mrad_s` | 角速度，mrad/s |
| `fault`、`valid`、`obstacle`、`sensor_fault` | 十六进制状态位 |

示例：

```text
TEL,t_ms=3180,tgt_l_mm_s=0,tgt_r_mm_s=0,spd_l_mm_s=0,spd_r_mm_s=0,pwm_l_x100=0,pwm_r_x100=0,enc_l=0,enc_r=0,odom_v_mm_s=0,odom_w_mrad_s=0,tof_l_mm=0,tof_r_mm=25,us_mm=412,ax_mg=11,ay_mg=-18,az_mg=1002,gx_mrad_s=2,gy_mrad_s=-1,gz_mrad_s=0,fault=0x0001,valid=0x000E,obstacle=0x0000,sensor_fault=0x0008
```

这里 `valid=0x000E` 表示右 ToF、超声波和 IMU 有效；左 ToF 无效。无效传感器即使显示 0，也不能解释成距离为 0。

## 5. UART4 可发送命令

串口工具发送结尾应选 CRLF 或 LF；命令使用小写 ASCII。

| 命令 | 作用 |
|---|---|
| `help` | 显示帮助 |
| `status` | 立即输出一行 TEL |
| `params` | 显示左右 PID、轮距、限速和看门狗参数 |
| `stream on` / `stream off` | 开关 5 Hz ASCII 自动输出 |
| `stop` | 立即清 PID 并停车 |
| `vel 0.05 0` | 设置车体线速度 0.05 m/s、角速度 0 rad/s |
| `wheel 0.05 0.05` | 设置左右轮闭环速度，单位 m/s |
| `pid both 0.05 0.001 0` | 修改左右 PID；`left`、`right`、`both` 可选 |

安全规则：

- `vel` 和 `wheel` 每次只刷新一次现有 500 ms 看门狗；用普通串口工具单发时，电机最多动作约 500 ms。
- 连续运动必须以约 20 Hz 重发，并在结束时发送 `stop`。建议使用下方脚本。
- `pid` 会先停车再修改，范围限制为 0–10；参数仅保存在 RAM，复位后恢复 `include/board_config.h` 中的值。
- RK 正在持续发送 `/cmd_vel` 时，“最后收到的命令”会覆盖调试命令。调试运动时应暂停 RK 的速度发布器；只看遥测可以同时进行。
- 首次电机测试必须轮子悬空、急停可用，并从 0.02–0.05 m/s 开始。

## 6. 电脑端用法

只监看 UART4：

```powershell
& "C:\Users\admin\.platformio\penv\Scripts\python.exe" `
  "G:\work\ROS2\Chassis\tools\ascii_debug_console.py" `
  --port COM19 --monitor
```

发送帮助或读取参数：

```powershell
python G:\work\ROS2\Chassis\tools\ascii_debug_console.py --port COM19 --command help --command params --duration 3
```

以 20 Hz 连续发送 0.05 m/s，持续 2 秒，脚本结束时自动发送 `stop`：

```powershell
python G:\work\ROS2\Chassis\tools\ascii_debug_console.py --port COM19 --linear 0.05 --angular 0 --duration 2
```

使用 USART1 二进制口测试 RK 协议：

```powershell
python G:\work\ROS2\Chassis\tools\closed_loop_test.py --port COM8 --baud 115200 --linear 0.05 --duration 2
```

两个脚本对应不同物理串口，不能把 ASCII 脚本连到 USART1，也不能把二进制脚本连到 UART4。
