# 2026-10-26 log_13 5–7 Hz 抖动分析与降振候选

## 结论

`log/26.10.26/2/log_13_UnknownDate.ulg` 确认为新固件日志：

- 时长约 55.6 s，连续在控约 51.9 s；
- `MOT_POLE_COUNT=28`；
- 上/下电机中位机械转速约 9.8k/9.3k rpm；
- 姿态中位倾角约 11.3°，95% 倾角约 16.1°。

主要抖动不是固定 5 Hz 结构共振，而是约 5–8 Hz、随时间移动的闭环极限环。

## 频谱特征

主要在控段的全局峰约为 6.35 Hz。

| 信号 | 4–6 Hz 功率占比 |
| --- | ---: |
| roll rate | 15.5% |
| pitch rate | 16.9% |
| yaw rate | 15.1% |
| servo 1 | 12.7% |
| servo 2 | 16.4% |
| servo 3 | 12.1% |

角速度与舵机命令的相干性：

- roll–servo1 最大约 0.98；
- pitch–servo3 最大约 0.94；
- 舵机1/2/3 达到 ±1 的时间比例约 13.3%/1.7%/10.2%。

角速度与舵机命令共同振荡、且在 5–8 Hz 间移动，说明优先怀疑控制增益、
舵机延迟/回差和饱和形成的极限环，而不是单纯 IMU 噪声。

## 修改

只改变 roll/pitch 角速度反馈强度：

```text
state_scales[2:4]: 0.2 -> 0.3
```

这降低了刚出现过冲的 p/q 反馈增益，同时保持 yaw、积分、舵机和电机权重不变。

## 验证

- 21 个日志窗口质量门控通过；
- 输入 `abs(z)` p95：1.9185；
- 本地辨识模型极点半径：0.993997；
- Python/C++ 标称和 19 状态黄金向量通过；
- 4 s 四方向末倾角：0.160–0.930°；
- 8 s 四方向末倾角：0.400–0.469°；
- ±roll 舵机反对称误差：约 0.00029；
- 极性、有限值、限幅和标称方向一致性通过。

模型：

```text
nominal checksum:     0xf1486f49
identified checksum:  0x1716d847
identified weight bytes: 580
```

固件：

```text
px4_trans/px4/build/micoair_h743-v2_nncontrol/micoair_h743-v2_nncontrol.px4
```

SHA-256：

```text
8b131d4264f6c5138f670b401753676c458f092dc2a0d355416efb319a3c9502
```

## 新增日志主题

保留原有五个标定核心主题的原生/500 Hz 记录，并增加：

| 主题 | 记录间隔 |
| --- | ---: |
| `vehicle_acceleration` | 5 ms |
| `sensor_accel` 0/1 | 10 ms |
| `vehicle_rates_setpoint`（可选） | 20 ms |
| `vehicle_attitude_setpoint`（可选） | 20 ms |
| `vehicle_local_position` | 50 ms |
| `battery_status`（可选） | 200 ms |
| `system_power` | 200 ms |
| `vehicle_control_mode` | 100 ms |

`vehicle_rates_setpoint` 和 `vehicle_attitude_setpoint` 只有在该控制模式实际发布时
才会出现在 ULog 中；其余新增主题用于下一轮区分结构振动、执行器极限环和高度变化。

## 下一轮判据

重新刷写后先确认：

```text
MOT_POLE_COUNT = 28
NN_LQI_MODEL = 1
nn_control checksum = 0x1716d847
```

下一次日志应重点比较：

1. 5–7 Hz 带内功率是否明显下降；
2. servo1/3 饱和比例是否从约 13%/10% 明显下降；
3. 回正是否仍能在数秒内完成；
4. 加速度频谱是否在同样频点出现峰值。

如果峰值仍在且加速度与陀螺同频，而舵机命令明显减弱，则转为结构共振排查；
如果舵机命令仍持续 ±1，则继续降低 p/q 反馈或增加最终舵机命令滤波。

本次降振数据、配置、模型、日志主题和固件归档：

```text
Identification/backups/20261026_pq03_damping_logging_candidate.tar.gz
```

SHA-256：

```text
c5eb77831306d5045c4f520880c9c64356aedcf3f76c534d3c02447e1d780b9b
```
