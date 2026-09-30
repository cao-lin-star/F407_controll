# STM32F407 小车底盘 — V2.0.0

**适配三路超声波，配套 RK3576 V3.0.0。** 发布标签：`V2.0.0`，日期：2026-09-30。

[F407 仓库](https://github.com/cao-lin-star/F407_controll) · [配套 RK V3.0.0](https://github.com/cao-lin-star/RK3576/tree/V3.0.0)

## 版本变动

| 版本 / Git 标签 | 日期 | 变动与配套关系 |
|---|---|---|
| **V2.0.0** | 2026-09-30 | 适配前、左、右三路超声波；左右保护总开关及回读；长回波处理；双 TOF 28cm 防跌落；近距有效测量与失联区分；保护恢复和 RK 新速度命令衔接。配套 **RK V3.0.0**。 |
| v1.0.1 | 2026-09-20 | 修改脱困恢复协议和命令处理，增加恢复状态配合；配套 RK v2.1.0。 |
| v1.0.0 | 2026-08-31 | 建立 STM32F407 可运行基线：FreeRTOS、双轮 PID、编码器里程计、PS2、前超声、双 TOF、MPU6050、正式串口与独立调试串口。 |

旧版本按原标签保留。本次只整理、发布当前源码，不加入尚未实现的 IMU 融合或打滑识别。

## 当前功能

- 100Hz 双轮控制，差速解算、编码器里程计、速度斜坡、PWM 输出和命令看门狗。
- 前/左/右超声轮询，保留前向优先采样及长回波处理，协议上报三路距离、有效状态和开关确认。
- 左右超声作为近障保护；F407 不做玻璃材质分类。停稳确认和地图补充障碍由 RK 完成。
- 双 TOFSense-M S 测距、防跌落和失联保护；最低可接受距离 **15mm（1.5cm）**，仍须满足有效区域数及协议状态条件。
- 保护期间停车；解除后通过零速确认及恢复握手接受新命令，不恢复旧速度。
- PS2、UART4 调试和 RK 命令仲裁；MPU6050 原始数据采集上报。

## 主要参数与接线

| 项目 | 当前值 |
|---|---|
| 前超声停车 / 解除 | 20cm / 28cm |
| 左、右超声停车 | 探头距离 ≤12cm；有效新鲜回波 >12cm 不触发侧向近障 |
| 左右超声安装 | 左前 / 右前各45°，沿波束内缩车体外沿约10cm |
| 双 TOF 防跌落 / 解除 | ≥28cm / ≤26cm；断线和无效测量另有保护 |
| 轮径 / 轮距 | 0.124m / 0.330m |
| 编码器 | 3468 count/rev（轮端四倍频） |
| 控制周期 / CMD_VEL 超时 | 10ms / 500ms |
| RK 正式串口 | USART1 PA9/PA10，115200 8N1，AA55 协议 v1 |
| ASCII 调试串口 | UART4 PC10/PC11，115200 8N1 |
| 左 / 右 TOF | USART2 PD5/PD6、USART3 PD8/PD9，921600 8N1 |
| 前超声 Trig / Echo | PC0 / PB6（TIM4_CH1） |
| 左超声 Trig / Echo | PC1 / PD13（TIM4_CH2） |
| 右超声 Trig / Echo | PC2 / PD14（TIM4_CH3） |
| MPU6050 | PB8/PB9 I²C1 |

参数以 [`include/board_config.h`](include/board_config.h) 和传感器实现为准。12cm 指探头测距，不是车体外沿净间距。F407/RK 信号接口按 3.3V TTL 接线，5V Echo 须按板级电气要求处理。

## 源码与构建

必须下载整个仓库；仅复制 `F407/` 会缺少共享控制源码。

- CubeMX：`F407/Footbath_Chassis_F407.ioc`
- Keil：`F407/MDK-ARM/Footbath_Chassis_F407.uvprojx`
- 应用分层：`F407/App/Business`、`Function`、`Hardware`
- 共用控制与协议：`include/`、`src/`
- 测试：`python test/run_tests.py`；CMake/CTest 见 `CMakeLists.txt`
- 固件环境：STM32Cube FW_F4 V1.28.3、Keil ARMCC 5.06 update 6。
- 烧录前断开24V电机电源，保留逻辑电源；更新后再验证传感器及方向。

[`docs/PROTOCOL.md`](docs/PROTOCOL.md) · [`docs/UART4_ASCII_日志与命令使用说明.md`](docs/UART4_ASCII_日志与命令使用说明.md) · [`docs/STM32F407_全外设与底盘调试步骤.md`](docs/STM32F407_全外设与底盘调试步骤.md)

## 发布来源与限制

本次源码对应最近一次构建固件，HEX SHA-256 与 RK 保存的 2026-09-28 18:09 部署固件一致：

`12c33fc195823afae2bf3996a85bab3edbd44291412b060973023072c55af443`

发布不重新烧录、不读取正在运行的 MCU Flash。源码核对和构建记录不能替代现场验收。
当前里程计仍基于编码器，IMU 仅采集上报；轮子打滑不等于真实位移，不能保证发现传感器盲区内的碰撞。
历史专题文档可能描述当时阶段行为，当前功能以本 README、配套版本及源码为准。
