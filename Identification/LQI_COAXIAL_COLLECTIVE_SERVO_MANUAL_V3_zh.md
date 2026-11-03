# 双电机总距/差模 + 三舵机姿态 LQI 手动标定 v3

## 目标

v3 基于 `flight2/7` 的实飞结果，解决三个明确问题：

1. 提高姿态指令跟踪速度，改善滚转/俯仰打杆手感；
2. 删除控制前的杆量一阶滤波，为后续接入外部姿态或位置外环减少无意义延迟；
3. 降低 PX4 日志采样量，避免 SD 写入导致 ULog dropout 和文件损坏。

v3 仍是仿真验证候选，不是飞行放行：

- controller：`lqi_coaxial_collective_servo`
- airframe：`airframe-1`
- model checksum：`0x78ae1111`
- `flight_authorized=false`

## 控制参数

物理参数沿用 manual-v2 的日志估计：

```text
mass              = 0.660 kg
inertia_roll      = 0.00050 kg m^2
inertia_pitch     = 0.00050 kg m^2
inertia_yaw       = 0.0007333333 kg m^2
servo_effectiveness_scale = 1.0
```

姿态状态尺度：

| 状态 | v2 | v3 |
| --- | ---: | ---: |
| roll/pitch 角度尺度 | 20° | 20° |
| roll/pitch 角速度尺度 | 1.4 rad/s | 1.4 rad/s |
| yaw 角速度尺度 | 3.0 rad/s | 3.0 rad/s |
| 位置积分尺度 | 0.20 | 0.20 |
| yaw 积分尺度 | 0.80 | 0.80 |

内环 P/D/I 保持不变。曾探索将角度与角速度反馈提高 22–39%，但滚转阶跃仿真
显示 90% 到达时间反而增加，原因是更早触发力矩限幅与舵机饱和。因此 v3
不采用“调大增益换取手感”的做法，指令响应提速由删除参考滤波实现。

## 参考输入

roll/pitch/yaw 的手动参考时间常数全部改为 `0.0`：

```text
roll stick  -> target roll      直接映射，20° 限幅
pitch stick -> target pitch     直接映射，20° 限幅
yaw stick   -> target yaw rate  直接映射，1.2 rad/s 限幅
throttle    -> target a_z       直接映射，±5 m/s²
```

生成器在时间常数为零时直接把 `manual_command_decay`写为 `0`，不引入滤波。
接入外环后，外环指令应直接写入同一参考接口。

## 日志压缩

上一段日志中，高采样率 PX4 ULog 造成：

- 153 次 dropout；
- 多个主题超过 20 ms 的数据间断，最长约 328 ms；
- `logger_sd_write` 单次平均约 0.40 s；
- 文件尾部零填充并报告损坏。

控制关键动态主题调整为：

| 主题 | v2 | v3 |
| --- | ---: | ---: |
| vehicle attitude | 500 Hz | 200 Hz |
| vehicle angular velocity | 500 Hz | 200 Hz |
| actuator motors | 500 Hz | 200 Hz |
| actuator servos | 500 Hz | 200 Hz |
| esc status | 500 Hz | 50 Hz |
| vehicle acceleration | 200 Hz | 100 Hz |
| sensor accel 0/1 | 100 Hz each | 50 Hz each |

这部分动态日志由约 `2900` 条/秒降到约 `1050` 条/秒，减少约 64%。
200 Hz 仍足以分析已出现的 3–8 Hz 刚体振荡；50 Hz RC、电池和估计器诊断
保持不变。

## 验证

20 个固定场景，4 s，500 Hz，模拟传感器：

| 指标 | v3 |
| --- | ---: |
| 安全率 | 100% |
| 回正末误差 p95 | 0.163° |
| 姿态跟踪末误差 p95 | 0.156° |
| 总距+姿态耦合末误差 p95 | 0.222° |
| 总距 z 加速度 RMSE | 0.0688 m/s² |
| 耦合 z 加速度 RMSE | 0.0687 m/s² |
| yaw 速率 RMSE | 0.00458 rad/s |
| 舵机饱和比例均值/最大 | 0.04% / 0.48% |

额外场景包含 20° 初始滚转/俯仰和 20° roll 指令，仍全部安全。探索性提高
P/D 的仿真没有缩短 90% 到达时间，因此最终 v3 保留 v2 内环增益。这不代替
真实系留测试。

原始报告：

`Identification/runs/lqi_coaxial_collective_servo_airframe1/validation_manual_v3.json`

## 产物

配置：

- `Controller/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml`
- `px4_trans/configs/lqi_coaxial_collective_airframe1_manual_v3.yaml`
- `Identification/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml`

模型：

- `px4_trans/generated/airframe-1/manual-v3/LqiCoaxialCollectivePayload.hpp`
- `px4_trans/generated/airframe-1/manual-v3/lqi_coaxial_collective_golden.hpp`
- `px4_trans/generated/airframe-1/manual-v3/lqi_coaxial_collective_golden.json`
- `px4_trans/generated/airframe-1/manual-v3/lqi_coaxial_collective_manifest.json`

固件：

`px4_trans/generated/airframe-1/manual-v3/micoair_h743-v2_nncontrol_manual_v3.px4`

SHA-256：

`0453336d9c3fd7694581955886145f2d54483635cb97ac60b6e215da1d366ff0`

## 使用边界

烧录后先确认：

```text
NN_LQI_MODEL = 2
NN_AIRFRAME_ID = 1
nn_control status 中 checksum = 0x78ae1111
```

无桨检查舵机方向、零位和全行程，然后进行低空系留测试。重点观察：

- 滚转/俯仰打杆是否在约 0.1–0.2 s 内出现明显角速度响应；
- 20°杆量是否出现超过约 2°的明显过冲；
- 舵机是否重新进入持续饱和；
- 新日志是否仍出现大段 dropout。

本版本不是飞行放行结论。
