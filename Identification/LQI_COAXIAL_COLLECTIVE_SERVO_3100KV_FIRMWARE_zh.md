# 机架 1 双电机总距 + 三舵机姿态固件（3100 kV 上桨空中标定）

## 结论

已按
`log/电机标定/3100kv上桨_空中.csv`
更新上桨空中 PWM–机械转速表，并保留上次机架 1 日志辨识得到的质量、惯量、
气动/反扭矩、舵机模型和辨识报告绑定，重新生成了：

- Python 控制器配置和 SimEnv 动力学；
- PX4 `NN_LQI_MODEL=2` 新控制核心及 8 状态姿态 LQI；
- 主机黄金向量和模型清单；
- `micoair_h743-v2_nncontrol` 固件镜像。

旧 `NN_LQI_MODEL=0/1` 均保留，不会被新模型覆盖。

## 电机表映射

固件输出边界 `FinalMotorOutputMap` 会把两路电机控制器命令再乘 `0.5`，而 CSV
记录的是最终物理执行器命令。因此 CSV 的 `command_normalized` 必须乘 2 才进入
控制器/SimEnv 命令域。

| 物理命令 | 控制器命令 | 实测 RPM | 机械转速 rad/s |
| ---: | ---: | ---: | ---: |
| 0.05 | 0.10 | 4268.45 | 446.991 |
| 0.10 | 0.20 | 5522.40 | 578.304 |
| 0.15 | 0.30 | 6850.775 | 717.411 |
| 0.20 | 0.40 | 7944.975 | 831.996 |
| 0.25 | 0.50 | 8940.025 | 936.197 |

CSV 只覆盖控制器命令 `0.10–0.50`。在 `0.50` 之后，模型从该实测点线性接到上次
已验证的满量程端点 `1200 rad/s`；这段不是新的实测结果。上下两桨均使用这张
相同的表，下桨复用是当前机架策略，不代表已经独立标定下桨空中转速。

上下桨使用完全相同的空中转速表。更新后的悬停配平：

- 上桨命令：`0.4393327`；
- 下桨命令：`0.4393327`；
- 两桨机械转速：`872.981 rad/s`；
- 总推力：`4.9033 N`；
- 姿态 LQI 极点半径：`0.995991`。

## 控制器与固件模型

新模型采用：

- 飞手杆量映射目标竖直加速度；
- 低通 z 加速度 PI 控制双电机共模；
- 双电机共模/差模解算总推力与 yaw；
- 8 状态姿态 LQI 输出 `[Mx, My, Mz]`；
- 三舵机只实现滚转/俯仰，并用整体向量缩放保持饱和时的零 yaw 方向。

PX4 `rc_throttle=-1/0/+1` 映射控制器总距 `0/0.5/1`：最低杆真正怠速，中心杆
对应零竖直加速度，满杆对应最大总距。

固件模型：

- checksum：`0xd5681c3f`；
- 权重/静态载荷：504 bytes；
- schema：`LqiCoaxialCollectiveModel::kSchemaVersion=3`；
- 辨识报告：`offline_identification_v7_hybrid_pq03.json`；
- 辨识报告 SHA-256：
  `344be729a1cfad46231b639d470b62ee0bb60c1216c130ae27042b34e0272571`；
- 上游辨识模型 checksum：`0x1716d847`；
- 标定 CSV SHA-256：
  `616a9a625f92593929f275180b1b43e6529fa0986b35fb8fbcf616a06f8f5d5d`。

## 构建与验证

生成载荷：

```bash
PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  px4_trans/tools/generate_coaxial_collective_backend.py
bash px4_trans/tests/run_lqi_core_check.sh
```

固件构建：

```bash
env CCACHE_DISABLE=1 MAKEFLAGS= \
  PATH="$PWD/px4_trans/px4/.venv/bin:$PATH" \
  PYTHON_EXECUTABLE="$PWD/px4_trans/px4/.venv/bin/python" \
  make -C px4_trans/px4 micoair_h743-v2_nncontrol
```

构建结果：

- Flash：`1089964 B / 1792 KB`，`59.40%`；
- AXI SRAM：`50760 B / 512 KB`，`9.68%`；
- 启动脚本：`NN_LQI_MODEL=2`；
- 参数默认值：`NN_LQI_MODEL=2`；
- 执行器输出门控：`NN_LQI_OUTPUT_EN` 默认仍为 `0`；
- Actuator metadata：`motor=2, servo=3`，检查通过。

主机检查：

```text
LQI core check PASSED (5 vectors)
identified LQI core check PASSED (5 vectors)
coaxial collective LQI core check PASSED (5 vectors)
```

## SimEnv 结果

重新运行 20 个 float32、500 Hz、2 s 工况，全部自动化门槛通过：

| 指标 | 新模型 | 旧五输出手动总距 LQR |
| --- | ---: | ---: |
| 安全率 | 100% | 100% |
| 姿态回正末误差 p95 | 0.271° | 0.595° |
| 姿态跟踪末误差 p95 | 0.358° | 0.424° |
| yaw 速率 RMSE 均值 | 0.0178 rad/s | 0.0754 rad/s |
| 总距 z 加速度 RMSE | 0.0805 m/s² | 0.0791 m/s² |
| 总距+姿态耦合末误差 p95 | 0.323° | 0.555° |
| 耦合场景 z 加速度 RMSE | 0.0854 m/s² | 0.1115 m/s² |
| 舵机饱和占比均值/最大 | 2.65% / 17.22% | - |
| 完整非线性舵机-yaw 残差最大值 | 5.07e-4 N·m | - |

原始报告：

```text
Identification/runs/lqi_coaxial_collective_servo_airframe1/validation_3100kv.json
```

## 固件产物

新固件：

```text
px4_trans/generated/airframe-1/micoair_h743-v2_nncontrol_coaxial_3100kv_airframe1.px4
```

- SHA-256：
  `4e63c79d8d3d61820f4b2655995f8faf8af73ea98b16946e1278e2937f53ee1f`
- 大小：`1062058` bytes

更新前固件备份：

```text
px4_trans/generated/airframe-1/prior-firmware/micoair_h743-v2_nncontrol_20261029_before_coaxial.px4
```

- SHA-256：
  `8e6c30017a9e4523fb4995f968787c88e20a929c5131d6dbabc2a899c83eccd2`

上一版“上桨新表、下桨旧表”固件：

```text
px4_trans/generated/airframe-1/prior-firmware/micoair_h743-v2_nncontrol_20261029_before_shared_upper_table.px4
```

- SHA-256：
  `c3459c4ceda158c846771f74b27af93710a92aebf79a97299369071791257199`

## 使用边界

这是编译和 SimEnv 验证通过的候选固件，不是飞行授权：

- 上桨实测只覆盖物理 5%–25%，更高段是明确标注的外推；
- 下桨按当前机架策略复用上桨空中表；仍应补一份独立下桨空中标定进行交叉验证；
- 尚未完成 HIL、无桨台架、系留、低空 shadow 和新构型真实闭环日志辨识；
- `NN_LQI_OUTPUT_EN` 必须保持 `0`，直到上述验证完成；
- 烧录后先确认 `NN_LQI_MODEL=2`、模型 checksum 和 `NN_AIRFRAME_ID=1`。
