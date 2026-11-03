# 双电机总距/差模 + 三舵机姿态 LQI 设计

> 2026-10-29 更新：上桨已改用
> `log/电机标定/3100kv上桨_空中.csv`，并生成 `NN_LQI_MODEL=2` 固件。最新参数、
> 验证和固件哈希见
> `Identification/LQI_COAXIAL_COLLECTIVE_SERVO_3100KV_FIRMWARE_zh.md`；本文后续
> 数值保留为上一版 2026-10-26 标定的设计记录。

## 1. 目标与隔离边界

本设计针对机架 1 的两个实测问题：

1. 上桨由飞手独占、下桨由程序控制，使 yaw 只能依赖下桨差速，正反方向权限
   不对称；
2. 旧五输出控制器允许电机参与姿态修正，总推力变化会经推力矢量舵机继续耦合到
   俯仰/滚转。

新结构注册为独立类型 `lqi_coaxial_collective_servo`，没有修改
`LQRController`、`HybridPIDLQRController`、`PIDController` 或成功实飞配置。
控制器仍输出 SimEnv/PX4 的五路物理命令：

```text
[upper_motor, lower_motor, servo_1, servo_2, servo_3]
```

输入所有权改为：

```text
飞手总距 -> 目标竖直加速度
                         |
                  z 加速度 PI
                         |
双电机共模 -> 总距      双电机差模 -> yaw

姿态 LQI -> 滚转/俯仰力矩 -> 三舵机约束混合
```

旧 `external_upper_4_output` 契约保持不变。

## 2. 控制结构

### 2.1 总距与竖直加速度

飞手杆量 `c` 在 `[0.20, 0.85]` 内线性映射为目标竖直加速度：

```text
a_cmd = 5.0 * [2 * (c - 0.20) / (0.85 - 0.20) - 1]
```

控制器对机体竖直方向总加速度做 10 ms 低通，并计算：

```text
a_filtered <- LPF(-linear_acceleration_n.z)
a_goal = a_cmd + Kp_z * (a_cmd - a_filtered)
                  + Ki_z * integral(a_cmd - a_filtered)
T_cmd = mass * (g + a_goal) / cos(tilt)
```

默认 `Kp_z=1.0`、`Ki_z=4.0`，积分限幅为 2 m/s²。总推力超限时冻结积分。
飞手中间杆量对应零竖直加速度，不接管高度位置；高度变化由飞手根据视觉/高度
信息主动控制。

### 2.2 双电机混合

机架 1 标称悬停点为上下电机均为 `0.4691509`、`872.981 rad/s`。控制器以
`balanced_motor_target(T_cmd)` 求共模基准，再围绕该点计算局部

```text
[dT, dMz] = J_motor [d_common, d_differential]
[upper_delta, lower_delta] =
    [[1, 1], [1, -1]] [d_common, d_differential]
```

总推力误差要求、yaw 力矩要求分别进入 `dT` 和 `dMz`。因此电机控制用途只有
总距共模和 yaw 差模。

### 2.3 三舵机姿态分配

姿态 LQI 状态为：

```text
[roll_error, pitch_error, p_error, q_error, r_error,
 integral(roll_error), integral(pitch_error), integral(yaw_rate_error)]
```

输入为 `[roll_moment, pitch_moment, yaw_moment]`。Riccati 方程从机架 1 的
`body.inertia_diagonal_b` 和 2 ms 控制周期合成，当前闭环极点半径为
`0.995991`。

滚转和俯仰力矩通过局部舵机 Jacobian 求解：

```text
min ||delta_servo||
s.t. J_servo * delta_servo = [0, roll_moment, pitch_moment, 0]
```

右端第一维为零保证舵机不承担共模推力，第四维为零保证舵机不产生额外 yaw。
初始化时从机架 1 的推力/反扭矩模型构建电机 `[T,Mz]` 局部权限表，运行时按
目标电机命令插值；舵机 Jacobian 在每个周期按目标总推力相对悬停总推力缩放。
这样既保留总距变化后的舵机权限归一化，也避免在 500 Hz 热路径中反复做完整
数值 Jacobian。任一舵机接近限幅时，三路舵机增量按同一个比例缩放，禁止逐通道
独立裁剪破坏零 yaw 共模方向。

## 3. 机架 1 参数复用

控制配置：

```text
Controller/configs/lqi_coaxial_collective_servo_airframe1.yaml
```

绑定来源：

- 动力学/执行器：
  `SimEnv/configs/sim2real_micro_coaxial_20261026.yaml`；
- 闭环辨识：
  `Identification/datasets/real_logs_26_10_26_v4/offline_identification_v7_hybrid_pq03.json`；
- 已部署辨识 LQI：
  `px4_trans/configs/lqi_airframe_1.yaml`；
- 校验和：`0x1716d847`。

电机表、质量、惯量、推力系数、舵机死区/回差/速率和零位来自上述机架 1
标定。验证脚本会检查辨识报告中的 airframe id 和嵌入模型校验和，避免误用
机架 2 数据。

## 4. 仿真验证

运行：

```bash
PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  -m flight_identification.coaxial_collective_evaluation \
  --config Identification/configs/lqi_coaxial_collective_servo_airframe1.yaml \
  --output Identification/runs/lqi_coaxial_collective_servo_airframe1/validation.json
```

协议：

- 500 Hz、`float32`、2 s、20 个固定场景；
- 4 个 5-10° 初始滚转/俯仰回正；
- 4 个 ±5° 姿态指令跟踪；
- 4 个 ±0.5/1.0 rad/s yaw 速率阶跃；
- 4 个 ±1/3 m/s² 总距竖直加速度阶跃；
- 4 个 ±5° 姿态指令与 ±3 m/s² 总距同时变化的耦合场景；
- 使用模拟陀螺、加速度计和电机转速传感器；
- 与旧五输出手动总距 LQR 在相同机架、初始状态和固定随机种子下配对运行。

结果（`simulation_validation_only`，`flight_authorized=false`）：

| 指标 | 新结构 | 旧五输出 LQR |
| --- | ---: | ---: |
| 场景安全率 | 100% | 100% |
| 姿态回正末误差 p95 | 0.348° | 0.551° |
| 姿态跟踪末误差 p95 | 0.306° | 0.348° |
| yaw 速率 RMSE 均值 | 0.0174 rad/s | 0.0756 rad/s |
| yaw 速率 RMSE 最大值 | 0.0277 rad/s | 0.1027 rad/s |
| 总距 z 加速度 RMSE 均值 | 0.0795 m/s² | 0.0772 m/s² |
| 总距+姿态耦合末误差 p95 | 0.297° | 0.498° |
| 总距+姿态耦合 z 加速度 RMSE | 0.0853 m/s² | 0.1140 m/s² |
| 舵机饱和占比均值/最大 | 2.68% / 17.22% | - |
| 局部模型舵机共模推力残差 | 0.0 N | - |
| 局部模型舵机-yaw 力矩残差 | 1.40e-9 N·m | - |
| 完整非线性舵机-yaw 残差最大值 | 5.06e-4 N·m | - |

新结构最直接的收益是 yaw 响应：平均 RMSE 降到旧方案的约 23%，最大误差降到
约 27%。姿态回正、姿态跟踪以及“总距+姿态”同时变化的场景也更接近目标。纯总距
场景的 z 加速度 RMSE 与旧方案相近，证明新增的双电机共模和 z 反馈没有以显著
竖直性能为代价；本次固定参数场景不能证明其在质量或推力标定失配下的鲁棒性。
表中前两项舵机残差是控制器所用局部线性混合模型的审计值。验证脚本另外用
完整 SimEnv 非线性舵机/格栅模型回算，最坏寄生 yaw 为 5.06e-4 N·m；它由 yaw
速率 LQI 闭环抑制，不是零残差。

## 5. 验证门槛与未完成项

本次自动门槛全部通过：

- 所有场景安全；
- 姿态回正/跟踪 p95 末误差 < 2°；
- yaw 速率 RMSE < 0.2 rad/s；
- 总距加速度 RMSE < 0.5 m/s²；
- 舵机推力/yaw 局部寄生量分别 < 1e-6 N 和 1e-5 N·m。

这些结果只证明机架 1 标称非线性 SimEnv 闭环可用。尚未完成：

- 质量、惯量、电机/舵机效能和时间常数的系统随机化；
- 控制器热路径在当前 WSL CPU、单线程下实测均值约 1.26 ms、p99 约 1.42 ms；
  原始未编译 `SimulationEnvironment.observe/advance` 全回路约 5.3 ms，因此
  500 Hz 实时部署时序仍需按目标 WebUI/PX4 路径单独验证；
- 电调反向、舵机安装方向、机械零位和死区在真机上的逐项核验；
- PX4 固件实现、黄金向量、SIL/HIL、无桨台架和系留飞行；
- 飞手总距中心与真实遥控器标定的一致性。

因此本控制器不得仅凭此验证进入实飞接管。
