# 验证记录（2026-08-14）

## 已完成

### 1. 资料核对与硬件配置

- 已从 `Docs` 中逐页核对 IBT-2、新款 HC-SR04/CS100A、TOFSense-M S、MPU6050 和野火 F407 霸天虎 V2 的资料。
- 电机参数已固化：24 V、51:1、17 PPR、电机轴 AB 相；TIM2/TIM3 四倍频后理论为 3468 count/rev。
- 轮径已更新为 124 mm，理论每计数 0.000112329149 m；轮距已更新为实测名义值 0.330 m。
- 板载 MPU6050 使用 PB8/PB9，固件接受 WHO_AM_I 0x68/0x70；同地址外接 GY-521 不可并联。
- TOFSense-M S 使用 USART2/3、921600 8N1，解析 4×4/112 B 和 8×8/400 B 主动帧，校验和通过且 `dis_status=0` 的像素才参与中值距离。
- HC-SR04 使用 12 μs Trig、80 ms 周期、70 ms Echo 超时，覆盖资料给出的无目标约 66 ms 高电平。

### 2. CubeMX 工程复现

- STM32CubeMX：6.17.1。
- STM32CubeF4：1.28.3。
- MCU：STM32F407ZGT6，LQFP144。
- 时钟：25 MHz HSE→PLL→168 MHz，APB1=42 MHz、APB2=84 MHz。
- 修复 `.ioc` 的 I2C1 模拟/数字滤波元数据后，已完成一次无界面重新生成。
- 重新生成后 `main.c` USER CODE 中的 `chassis_app_init()`、`chassis_app_process()` 以及 Keil 自有 App 文件组均被保留。

### 3. Keil 最终实编译

```text
Compiler: ARMCC 5.06 update 6 build 750
Target: Footbath_Chassis_F407
Program Size: Code=28548 RO-data=1760 RW-data=160 ZI-data=8368
Result: 0 Error(s), 0 Warning(s)
Artifact: F407/MDK-ARM/Footbath_Chassis_F407/Footbath_Chassis_F407.{axf,hex}
Log: F407/MDK-ARM/build_ps2_final.log
```

实际参与构建的自有模块包括 F407 App、板级驱动、传感器、控制、PID、运动学、协议和 CRC。

### 4. Windows 主机测试

```text
PASS test_crc
PASS test_fragment_sticky_and_noise
PASS test_crc_length_version_resync
PASS test_payload_layouts
PASS test_kinematics_and_old_speed_equivalence
PASS test_pid_reference_and_anti_windup
All 6 Python host tests passed.
```

新增 `tools/closed_loop_test.py` 已通过 Python 语法编译检查，可发送 CMD_VEL/STOP、解析 CRC 后的 64 字节 DEBUG_STATUS、输出 CSV，并验证 500 ms 命令看门狗。

### 5. 接口一致性

- UART：115200 8N1，3.3 V TTL。
- SOF：AA55；协议版本：1；CRC16-CCITT-FALSE 标准向量：0x29B1。
- 正式消息 0x01–0x06 未改变；新增的 0x7F 只用于可选诊断。
- 500 ms MCU 独立命令看门狗保留。
- PS2只在F407内部产生速度目标，USART1消息0x01～0x7F的布局均未改变；RK3576协议兼容。
- PS2以40 Hz轮询；只有手动模式且R1按住时占权，R1释放在一个轮询周期内立即停车并释权，SELECT立即停车。
- 即使PS2接收器在手柄关机后继续返回`0x73`和中位摇杆，UART4/RK也能接管；真实连续100 ms无合法帧仍会清除手动模式。
- 已只读核对 RK3576 当前实现：最大 payload 1024 字节，可接收本次 64 字节帧；未知 0x7F 仅增加计数并安全忽略，不会触发 STOP 或破坏流解析。

### 6. 接线图视觉检查

- `Docs/STM32F407_底盘接线图.svg` 已按 1600×1220 实际渲染检查。
- 图中包含 24 V/5 V 星形供电、两块 IBT-2、两个编码器、两个 ToF、HC-SR04、板载 MPU6050、RK/电脑二选一串口及首次上电顺序。
- 电机电源、逻辑电源、共地、5 V Echo 降压和 CH340/RK TX 冲突警告均已标出。

## 已知未完成项

- 尚未连接目标 F407 板执行 DAP 实物下载和断点调试。
- 尚未实测初始 PWM/EN、电机方向、编码器电平、每圈 3468 计数和闭环负反馈极性。
- 尚未在 24 V 实际负载下验证启动/堵转电流、温升、保险规格和 500 ms 停机距离。
- 124 mm 轮径对应的理论轮周和 0.330 m 尺量轮距仍需负载落地/多圈旋转标定，PID 只是悬空安全起点。
- PS2新增代码与CubeMX PA4～PA7配置已通过Keil ARMCC构建（0 Error、0 Warning），但尚未连接实物接收器验证模式ID、时序、按键、摇杆方向和无线失联。
- TOFSense仍由HAL逐字节中断取数，但ISR只把字节放入每路1024字节环形缓冲；整帧校验、64区排序和中值计算已经移到2 ms传感器任务。UART4用`tof_l_rxerr/tof_r_rxerr`观察累计错误；若修复后921600仍持续丢帧，再升级DMA/IDLE。
- `FRONT_OBSTACLE_SAFETY_ENABLE=1`；`CLIFF_SAFETY_ENABLE=1`，暂定左/右地面基线0.155/0.160 m，250 ms失联按台阶处理。两类联锁只阻止含前向轮速的命令，零速确认后允许低速倒退。
- ToF暂定基线和HC-SR04误检范围尚未完成真实场景验收；烧录后必须先做24 V关闭、悬空和低速测试，未通过前不得无人值守自动探索。
- 尚未完成 MPU6050 零偏/温漂、真实 C1 `/scan`、SLAM、AMCL、Nav2、EMI、涉水绝缘、防夹和整机安全验收。

## 下一次硬件验收顺序

1. 断开 24 V，只上逻辑电并 DAP 烧录。
2. 确认 PWM=0、EN=0、急停有效和串口零速诊断。
3. 手转左右轮，确认前进为正且每圈约 3468 count。
4. 轮子悬空执行 0.02 m/s、0.3 s 短脉冲，验证电机方向和负反馈。
5. 依次做 0.05/0.10 m/s 阶跃、STOP、拔 TX 和 500 ms 断链硬停。
6. 逐个接入 MPU6050、HC-SR04、左/右 ToF，核对有效位、距离和故障位。
7. 24 V关闭时接入PS2，验证`link=1`、START/R1/SELECT；松开R1后`source`必须立即离开3，并允许UART4/RK新命令接管。
8. 关闭手柄后若接收器仍报告`link=1, mode=0x73`，确认`r1=0`且UART4 `vel`仍返回accepted；若真实失联，则100 ms内应为`link=0, manual=0`。
9. UART4 `stop`后确认PS2手动使能被取消，需重新按START才能再由PS2运动。
10. 负载落地标定左右轮周、轮距和 PID。
11. 验证暂定ToF基线、断线前向停车、零速确认后倒退和HC-SR04阈值；需要调整时先改参数并重新编译，不得跳过验证直接自动探索。
12. 最后连接 RPLIDAR C1 与 RK3576，完成 `/scan`、TF、SLAM、地图保存/加载、AMCL 和 Nav2。
