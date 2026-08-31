# 从旧 ESP32 工程迁移的依据

本次迁移实际读取了旧工程以下文件，而不是只依据口头参数：

- `src/main.cpp`
- `lib/Kinematics/Kinematics.h/.cpp`
- `lib/PidController/PidController.h/.cpp`
- `platformio.ini`

旧工程路径：`/home/sky/chapt9/fishbot_motion_control`（WSL Ubuntu-22.04）。读取时旧源码中的
一部分中文注释存在编码乱码，但代码常量和运算明确可辨。

## 已原样保留的行为/参数

| 项目 | 旧 ESP32 实现 | STM32 首版 |
|---|---|---|
| 轮距 | `set_wheel_distance(175)` mm | `WHEEL_TRACK_M = 0.175` m |
| 每 tick 距离 | `set_motor_param(..., 0.105805)` mm/tick | `0.000105805` m/tick |
| 左右 PID | `kp=0.625, ki=0.125, kd=0` | 同值，仍按固定周期离散积分 |
| PID 输出 | `[-100,100]` | `[-100,100]` 百分比 |
| 积分限值 | 旧类默认 `2500` | `2500`，另增加条件积分抗饱和 |
| CMD_VEL 超时 | 500 ms | 500 ms，有效 CRC 帧才刷新 |
| 控制周期 | `loop()` 内 `delay(10)`，约 10 ms | HAL tick 固定 10 ms 调度 |
| ODOM 发布 | micro-ROS timer 50 ms | UART ODOM 50 ms（20 Hz） |

## 速度公式的单位复核

旧 `Kinematics.cpp` 不是直接使用成员 `per_pulse_distance`，而是写死：

```text
motor_speed = delta_ticks * 105.805 / dt_ms
```

因为 `0.105805 mm/tick × 1000 ms/s = 105.805`，上式结果确实是 mm/s。例如 10 ms 内
10 tick，速度是 `105.805 mm/s`。STM32 代码用 SI 常量计算：

```text
speed_mm_s = delta_ticks × 0.000105805 m/tick × 1000 mm/m / 0.010 s
```

结果完全相同。PID 继续使用 mm/s 误差，因此旧增益的量纲没有被悄悄改变；对外 UART 和 ODOM
则统一使用 m/s、rad/s、m。

## 明确改变的部分

1. 去掉 ESP32 Wi-Fi、micro-ROS、FreeRTOS 双核共享和 epoch 同步；RK3576 改用 115200 8N1
   有线 UART 二进制协议。
2. 旧 PID 的积分只做数值截断；新版本同时使用积分限值和“输出饱和且误差继续推向饱和时暂停积分”。
3. 旧 PID 导数写成 `prev_error - error`，但旧配置 `kd=0`，所以历史行为不受影响。新实现采用常见的
   `error - previous_error`；若以后启用 kd，必须重新整定。
4. 旧里程计用更新后的航向计算整段位移；新实现使用区间中点航向积分，转弯时误差更小。
5. 新增 STOP 立即停机、CRC/长度验证、无动态内存、UART 中断环形缓冲、编码器跳变/堵转检测、
   调度超期保护和 fault_flags。
6. 旧 `platformio.ini` 是 `esp32dev + Arduino + micro_ros_platformio + Wi-Fi`；新工程是
   `bluepill_f103c8 + STM32Cube HAL + ST-Link`，不依赖旧私有库或未生成代码。

## 仍需实车重新确认

旧参数只证明软件历史值，不证明它们适合新电机、编码器安装或 STM32 定时器的 x2/x4 计数方式。
尤其要实测“手转一圈的累计 tick”，再校准每 tick 距离；随后在轮子悬空和低速落地两阶段重新整定 PID。
