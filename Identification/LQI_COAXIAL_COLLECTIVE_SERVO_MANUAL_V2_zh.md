# 双电机总距/差模 + 三舵机姿态 LQI 手动标定 v2

## 状态

本版本是 `flight2/6` 两段实飞日志驱动的保守手动标定候选：

- 控制器：`lqi_coaxial_collective_servo`
- 机架：`airframe-1`
- 模型 checksum：`0x5c2d17af`
- SimEnv 固定场景门槛：全部通过
- 发布状态：`simulation_validation_only`
- 飞行授权：`false`

当前没有精确参数辨识，因此本版本不声称已获得真实惯量、舵机有效力矩
或时间常数。它只使用日志中重复出现、量级稳定的信息，将控制器从明显不匹配的
保守初值移向实测响应。

## 日志依据

分析输入：

- `log/flight2/6/log_12_2026-10-30-10-04-04.ulg`
- `log/flight2/6/log_13_2026-10-30-10-04-34.ulg`

关键结论：

- 两段均为 `NN_LQI_MODEL=2`、`NN_LQI_OUTPUT_EN=1`、机架 1；
- `nn_control` 实测约 500 Hz，没有 1.5 ms/2 ms 周期超时；
- 姿态振荡主峰约 `3.4-4.7 Hz`，舵机命令到角加速度的方向正确，但延迟约
  `55-67 ms`；
- 在控飞行段中，至少一个舵机接近饱和的时间约占 70%；
- 使用 ESC 转速、现有推力曲线、姿态倾角和局部垂向加速度，反推等效质量约为
  `0.65-0.68 kg`；
- `vehicle_acceleration` 与位置估计的垂向加速度没有明显反号或零滞后偏差，
  主要问题是高频动态误差和噪声，不是总距迟钝的首要原因。

## 手动参数

| 参数 | 原值 | v2 |
| --- | ---: | ---: |
| 机体质量 | `0.500 kg` | `0.660 kg` |
| roll/pitch 惯量 | `0.00225 kg m^2` | `0.00050 kg m^2` |
| yaw 惯量 | `0.00330 kg m^2` | `0.0007333333 kg m^2` |
| 姿态状态尺度 | `1x` | `2x` |
| 舵机效能标定系数 | `0.80` | `1.00` |
| 总距 z 加速度 PI | `Kp=1, Ki=4` | 不变 |
| z 加速度低通时间常数 | `10 ms` | 不变 |

惯量取日志估计的偏高侧，而不是最小估计值，以降低实际车辆响应更强时的
过激控制风险。姿态状态尺度放大两倍，将 roll/pitch/yaw 的期望力矩增益
降低约一半，为已观测到的舵机相位滞后留出裕量。

总距前馈改为

```text
T_cmd = 0.660 * (g + a_cmd) / cos(tilt)
```

使推力前馈从原先约缺少 24% 改为与日志反推有效载荷一致。没有提高
`maximum_acceleration_m_s2`，也没有扩大力矩限幅。

## 验证结果

配置：

- `SimEnv/configs/sim2real_micro_coaxial_3100kv_airframe1_manual_v2.yaml`
- `Controller/configs/lqi_coaxial_collective_servo_airframe1_manual_v2.yaml`
- `Identification/configs/lqi_coaxial_collective_servo_airframe1_manual_v2.yaml`

20 个固定场景，4 s，500 Hz，模拟传感器：

| 指标 | v2 |
| --- | ---: |
| 安全率 | 100% |
| 回正末误差 p95 | 0.163° |
| 姿态跟踪末误差 p95 | 0.156° |
| 总距+姿态耦合末误差 p95 | 0.222° |
| 总距 z 加速度 RMSE | 0.0688 m/s² |
| 耦合 z 加速度 RMSE | 0.0687 m/s² |
| yaw 速率 RMSE | 0.00458 rad/s |
| 舵机饱和比例均值/最大 | 0.04% / 0.48% |
| 完整非线性舵机-yaw 残差最大值 | 3.03e-4 N·m |

原始报告：

`Identification/runs/lqi_coaxial_collective_servo_airframe1/validation_manual_v2.json`

探索性偏差测试中，固定 v2 控制器面对：

- 质量 `0.60-0.72 kg`；
- roll/pitch 惯量 `0.00035-0.0010 kg m^2`；

仍保持安全。模拟把三舵机时间常数直接放大到 `0.12 s` 时会失稳；这比日志
反推的约 `60 ms` 慢一倍，说明本候选仍不能替代系留测试，也不能证明存在
任意大执行器延迟时的鲁棒性。

## 固件产物

模型和黄金向量：

- `px4_trans/generated/airframe-1/manual-v2/LqiCoaxialCollectivePayload.hpp`
- `px4_trans/generated/airframe-1/manual-v2/lqi_coaxial_collective_golden.hpp`
- `px4_trans/generated/airframe-1/manual-v2/lqi_coaxial_collective_golden.json`
- `px4_trans/generated/airframe-1/manual-v2/lqi_coaxial_collective_manifest.json`

固件：

`px4_trans/generated/airframe-1/manual-v2/micoair_h743-v2_nncontrol_manual_v2.px4`

SHA-256：

`c83b44c4d9f51bb980fd6debd66a37fd46bc4468d90c9c78139f668449c5f6bf`

主机 LQI 黄金向量、Python 单元测试和 PX4 固件编译均通过。

## 日志改进

参数辨识日志新增：

- `manual_control_setpoint`，20 ms；
- `battery_status`，200 ms；
- `estimator_status`，200 ms；
- `estimator_sensor_bias`，200 ms；
- `sensors_status_imu`，200 ms。

这可以在下一段日志中直接区分飞手输入与闭环振荡，并检查总距推力漂移是否来自
电池电压或加速度偏置。

## 使用边界

在烧录或飞行前仍需：

1. 核对固件 SHA-256 和 `nn_control status` 中的模型 checksum
   `0x5c2d17af`；
2. 无桨检查三舵机方向、零位和最大行程；
3. 完成低空系留或限高 shadow 测试，确认 3-5 Hz 振荡和舵机饱和下降；
4. 保持 `NN_LQI_OUTPUT_EN=0` 直到上述检查完成。

本版本不是飞行放行结论。
