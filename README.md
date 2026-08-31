# STM32F407 两轮差速底盘

本工程面向已购的野火 STM32F407ZGT6 霸天虎 V2、2×IBT-2/BTS7960、2×24 V 51:1 编码减速电机、124 mm 车轮、板载 MPU6050、新款 HC-SR04/CS100A 和 2×TOFSense-M S。STM32 负责 10 ms 双轮闭环、编码器、里程计、传感器采集和独立安全停机，通过有线 UART 与 RK3576 ROS2 通信。

独立 Git 仓库：`https://github.com/cao-lin-star/F407_controll.git`。仓库根必须是整个 `Chassis`，因为 Keil 工程同时引用 `F407/`、根目录 `include/` 和 `src/`。

## 工程入口

- CubeMX：`F407/Footbath_Chassis_F407.ioc`
- Keil：`F407/MDK-ARM/Footbath_Chassis_F407.uvprojx`
- F407 业务层：F407/App/Business；功能层：F407/App/Function；硬件层：F407/App/Hardware
- 共享控制核心：`include`、`src`
- 二进制协议调试：`tools/closed_loop_test.py`
- ASCII 调试口：`tools/ascii_debug_console.py`
- 协议：`docs/PROTOCOL.md`
- 双串口与十六进制说明：`docs/SERIAL_DEBUG_GUIDE.md`
- PS2接线与控制：`docs/PS2无线手柄接线与控制说明.md`
- 参数实现：`include/board_config.h`（修改参数时同步 README 和相关测试）
- F407 全外设接线与测试：`docs/STM32F407_全外设与底盘调试步骤.md`
- 正式协议：`docs/PROTOCOL.md`
- RK/F407 跨板接线：`https://github.com/cao-lin-star/RK3576/blob/main/docs/RK3576与STM32F407接线表.md`

推荐流程是 CubeMX 配置外设并生成 MDK 工程、运行 F407/tools/relink_keil_freertos.ps1 恢复分层和 FreeRTOS 组、VSCode 编写应用代码、Keil 编译/烧录/调试。必须保留 main.c USER CODE 区内的 chassis_app_init() 与 chassis_tasks_start()。

## 当前硬件参数

| 项目 | 当前值 |
|---|---:|
| 电机 | 24 V、51:1、额定 190 rpm、额定 3.5 N·m、堵转电流不高于约 21 A/只（卖家值） |
| 编码器 | AB 双相增量式霍尔、17 PPR、电机轴安装 |
| 轮端四倍频计数 | 17×51×4 = 3468 count/rev |
| 轮径 | 0.124 m |
| 理论每计数距离 | 0.000112329149 m；必须落地复标 |
| 轮距 | 0.330 m，已按实际尺量值写入，仍需旋转标定 |
| 控制周期 | 10 ms / 100 Hz |
| PWM | TIM1，20 kHz |
| 初始 PWM 上限 | 35% |
| CMD_VEL 硬超时 | 500 ms |
| PS2轮询/失联停车 | 40 Hz / 100 ms |
| 控制优先级 | PS2 > UART4 > RK；高优先级断流后自动释权 |
| 单轮速度斜坡 | 加速0.30 m/s²，减速0.50 m/s²；安全停车绕过 |
| IBT-2零速 | PWM=0、EN关闭；动态电制动默认不启用 |

## 引脚

| 功能 | 引脚/外设 |
|---|---|
| 左 IBT-2 RPWM/LPWM/EN | PE9 TIM1_CH1、PE11 TIM1_CH2、PE8 |
| 右 IBT-2 RPWM/LPWM/EN | PE13 TIM1_CH3、PE14 TIM1_CH4、PE10 |
| 左编码器 A/B | PA15 TIM2_CH1、PB3 TIM2_CH2 |
| 右编码器 A/B | PB4 TIM3_CH1、PB5 TIM3_CH2 |
| RK3576 正式串口 | PA9 TX→RK物理3脚 UART3_RX_M0；PA10 RX←RK物理5脚 UART3_TX_M0；115200 8N1 |
| PS2无线接收器 | PA4 DATA、PA5 CMD、PA6 ATT/CS、PA7 CLK，3.3 V软件时序 |
| 左 ToF | PD5/PD6 USART2，921600 8N1 |
| 右 ToF | PD8/PD9 USART3，921600 8N1 |
| 板载 MPU6050 | PB8/PB9 I²C1，0x68，WHO_AM_I 0x68/0x70 |
| 超声波 | PC0 Trig、PB6 TIM4_CH1 Echo |
| 红外预留/急停 | PG2、PG3、PG4，低有效 |
| 状态灯 | PF6，低有效 |
| 独立 USB-TTL ASCII 调试串口 | PC10/PC11 UART4，115200 8N1 |
| fireDAP/SWD | PA13、PA14；另接 NRST、VTref、GND，TX/RX不接 |

HC-SR04 若使用 5 V，Echo 必须先降为 3.3 V 后进入 PB6。USART1 专用于 RK3576 二进制协议；UART4 如需 ASCII 遥测，使用独立 3.3 V USB-TTL 模块。fireDAP 只承担 SWD，不能再兼任串口。PC10/PC11 与 SD 卡 DAT2/DAT3 复用，调试时不能插 SD 卡。USART1 连接 RK 前要移除或隔离板载 CH340 串口跳帽，避免两个 TX 同时驱动 PA10。RK 与 F407 只接交叉 TX/RX 和 GND，不接电源。霸天虎 V2 已有板载 MPU6050，不能再并联地址同为 0x68 的 GY-521。

## 软件模块

- F407/App/Business/Src/chassis_tasks.c：FreeRTOS 四静态任务和故障钩子。
- F407/App/Business/Src/chassis_app.c：命令仲裁、协议、遥测时序与安全联锁。
- F407/App/Function/Src/ps2_remote.c：PS2 软件时序、摇杆映射和失联停车。
- F407/App/Function/Src/sensor_hub.c：MPU6050、双 ToF、超声和障碍标志。
- F407/App/Function/Src/debug_service.c：ASCII 遥测、速度/PWM命令和运行时 PID。
- F407/App/Hardware/Src/board_f407.c：PWM、IBT-2 EN、编码器和寄存器级硬停。
- F407/App/Hardware/Src/uart_transport_f407.c：USART1 环形缓冲和异步 TX。
- F407/App/Hardware/Src/debug_console.c：UART4 ASCII 非阻塞收发。
- src/control.c：PID、差速目标、500 ms 看门狗和里程计。
- src/protocol.c：AA55 协议流解析与构帧。
传感器驱动和诊断默认启用；正前超声通过`FRONT_OBSTACLE_SAFETY_ENABLE=1`、左右ToF通过`CLIFF_SAFETY_ENABLE=1`参与F407方向性停车联锁，仅阻止含前向轮速的命令。ToF暂定左/右基线0.155/0.160 m且失联250 ms按台阶处理；烧录后必须验收平地、断线、真实台阶与零速确认后的倒退恢复。红外由`IR_OBSTACLE_SAFETY_ENABLE=0`保持关闭。

### 可恢复停机

编码器跳变/堵转、急停、障碍、控制超期和非法命令均先立即停机，但不再要求 MCU 复位。恢复必须包含零速度或 `STOP` 确认；编码器还要求双轮连续稳定 500 ms，通信类故障要求连续 1 s 无同类错误。解除后不会续跑旧速度，必须重新发送非零命令。详见 `docs/PROTOCOL.md`。


## 构建与测试

最终实编译环境：STM32CubeMX 6.17.1、STM32Cube FW_F4 V1.28.3、Keil ARMCC 5.06 update 6。

```text
0 Error(s), 0 Warning(s)
Code=32520, RO-data=2116, RW-data=248, ZI-data=20216
```

主机回归测试：

```powershell
python test\run_tests.py
```

台架诊断与速度命令：

```powershell
python tools\closed_loop_test.py --port COM8 --linear 0.05 --duration 3
```

## 首次上电底线

1. 先断开 24 V 电机电源，只烧录和验证逻辑电。
2. 轮子悬空，确认四路 PWM 初始为 0、两个 EN 为停机状态。
3. 手转每个轮子一圈，应接近 3468 计数且前进方向左右均为正。
4. 只发 0.02 m/s、0.3 s 的短命令，随后 STOP，确认两轮方向和负反馈闭环。
5. 停止发送或拔掉上位机/RK TX，最迟 500 ms 必须停机。
6. 24 V关闭时先验证PS2 `link=1`、模拟模式、START/R1/SELECT和100 ms失联停车。
7. 完成轮周、轮距、PID、ToF 地面基线和传感器误检标定后，才允许提高 PWM 上限或打开安全联锁。

当前结论表示 FreeRTOS 固件编译（0 Error、0 Warning）、Python 12/12 和 CTest 1/1 通过；可恢复故障状态机仍待轮子悬空回归，不代表已通过真实负载、堵转、EMI、涉水、急停或整机安全验收。
