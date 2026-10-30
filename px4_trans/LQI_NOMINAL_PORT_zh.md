# Nominal LQI PX4 移植契约

当前部署目标是 `Controller/configs/lqi_sim2real_micro_coaxial_4out.yaml` 的
4×13 LQI，不是蒸馏实验中仍存在的 5 输出教师控制器。

## 执行器所有权

- 实机四轴手动量只接受物理 RC；地面站指令（GS-Link v1）只能经
  `GroundStationGate` 在 RC 解锁、通道 8 模式槽 6、External Mode 激活和指令新鲜度同时满足时替换
  四轴指令。详见 `GROUND_STATION_POSITION_LINK_zh.md`；目标串口协议见
  `GS_LINK_PROTOCOL_V1_zh.md`。
- 飞手的 `manual_control_setpoint.throttle` 从 `[-1,1]` 映射到 `[0,1]`，在控制器
  指令域内作为 `motors[0]` 上桨命令由 LQI 原样传递。
- 所有控制器后端与输入来源完成计算后，上桨和下桨的最终归一化输出统一乘以
  `0.5`，即控制器域满油门对应最终 `50%`；三路舵机不缩放。该映射不进入 LQI
  内部状态，因此权重、观察器和上下桨差限幅仍使用原标定尺度。
- 下桨的基准开度跟随同一个飞手开度，LQI 的第 0 路增量叠加到该基准后限幅到
  `[0,1]`。
- 油门空闲门控：映射后的飞手开度为 `0` 时，LQI 不计算控制输出、不更新持久
  状态，上桨/下桨/三路舵机全部输出 `0`；只有油门出现非零值后才开始输出。
- 上下桨开度差限制：下桨归一化开度被限制在飞手上桨开度 `±0.2`（20 个百分点）
  内，并同时保持在 `[0,1]`。
- LQI 的第 1～3 路直接作为三路舵机归一化命令，限幅到 `[-1,1]`。
- PX4 输出驱动负责实际 PWM 范围、方向、disarmed/failsafe 值；控制器不重复应用
  物理方向或 trim。

## 状态与参考系

13 维状态顺序固定为：

```text
[roll_error, pitch_error, p_error, q_error, r_error,
 upper_motor_error, lower_motor_error,
 servo_1, servo_2, servo_3,
 integral_roll, integral_pitch, integral_yaw_rate]
```

- 世界系为 NED，机体系为 FRD。
- 姿态为 Hamilton 四元数 `[w,x,y,z]`，表示机体 FRD 到世界 NED 的旋转 `q_wb`。
- 姿态误差严格使用 `conjugate(q_target_wb) * q_current_wb`，即与 Python
  `flight_controller.math.quaternion_rotation_error` 相同的 target-to-current
  局部旋转向量；只取 roll/pitch 两项，偏航采用角速度模式。
- 角速度和目标角速度都在机体 FRD 中，误差为 current-minus-target。
- PX4 正 pitch 杆表示向前推杆/机头下压，因此转换到 FRD Euler 目标时必须取负号；
  正 roll 杆保持正号。
- roll/pitch/yaw 指令使用部署清单中的一阶滤波；目标 yaw 从当前航向初始化并积分
  yaw-rate 指令，避免偏航过程中 roll/pitch 误差轴错位。
- `q` 和 `-q` 必须产生完全相同的控制结果，主机测试已覆盖该约束。

## 电机与舵机状态

电机目标转速使用与 Python 控制器一致的 PWM→rad/s 分段线性表。若
`esc_status` 中 Motor1、Motor2 的新鲜反馈同时可用，则按 actuator function 匹配、
对机械 RPM 取绝对值并转换为 rad/s；否则两路一起回退到命令驱动的一阶观察器。
舵机没有角度反馈，始终使用命令驱动观察器。

## 框架与权重分离

- `LqiControllerModel.hpp`：稳定的数据 ABI、维度和参考系/所有权枚举。
- `LqiControllerCore.hpp`：不包含任何具体权重的通用 500 Hz LQI 算法。
- `LqiManualReference.hpp`：PX4 摇杆到目标姿态/角速度的适配层。
- `LqiNominalModel.hpp`：生成的 nominal 数值载荷，禁止手改。
- `px4_trans/configs/lqi_nominal_deployment.yaml`：模型来源和命令尺度清单。

修改权重来源或命令尺度后，从仓库根目录重新生成并验证：

```bash
PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
  .venv/bin/python px4_trans/tools/generate_lqi_backend.py
bash px4_trans/tests/run_lqi_core_check.sh
```

生成器会同时更新 checksum、权重字节数以及 Python/C++ 黄金向量。模型描述中的
schema、NED/FRD、四元数约定、执行器所有权或观察器模式不匹配时，核心会拒绝运行。

### v2 标称权重（2026-08-28）

`px4_trans/configs/lqi_nominal_deployment_v2.yaml` 对应
`Controller/configs/lqi_sim2real_micro_coaxial_4out_v2.yaml`：

- roll/pitch 积分权重从 `0.04/0.04` 提升到 `0.025/0.025`（约 2.6 倍），积分限幅从
  `0.30/0.30` 提升到 `0.45/0.45`，用于补偿机械零点引起的 roll/pitch 持续稳态误差；
- 最大 roll/pitch 指令从 `12 deg` 提升到 `20 deg`（`0.3490659 rad`），yaw 速率指令不变；
- 其余 LQI 权重与 v1 4 输出配置一致。

重新生成并写入固件默认 nominal 载荷：

```bash
PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
  python px4_trans/tools/generate_lqi_backend.py \
  --deployment-config px4_trans/configs/lqi_nominal_deployment_v2.yaml
bash px4_trans/tests/run_lqi_core_check.sh
```

机械零点的底层输出补偿仍由 `NN_SV_TRIM1/2/3` 参数在 PX4 输出层完成；v2 矩阵的
积分增强处理的是残余偏置与收敛速度。辨识版 19 状态的复合响应系数不随本配置
改变，但重新生成载荷时会从所选部署配置刷新电机标定表、配平点、积分限幅和
手动参考尺度。

## 辨识模型版本（lqi_identified，19 状态复合 LQI）

2026-08-12 起，固件支持第二版 LQI 权重：由真机日志离线辨识得到的 19 状态复合
LQI。原有 13 状态标称权重（`LqiNominalModel.hpp`、`lqi_golden.*`）保持不动，
通过参数 `NN_LQI_MODEL` 选择：

- `NN_LQI_MODEL=0`：标称 13 状态 LQI（可运行选择）；
- `NN_LQI_MODEL=1`：真机辨识 19 状态复合 LQI（`lqi_identified`）。

当前 direct-controller 固件的启动脚本会在 `nn_control start` 前设置
`NN_LQI_MODEL=1`，因此默认运行 19 状态控制器；参数仍可在运行时改回 `0`，
但修改后需要重启 `nn_control`。执行器输出仍由 `NN_LQI_OUTPUT_EN` 统一门控。

19 状态顺序：

```text
[roll_error, pitch_error, p_error, q_error, r_error,
 upper_motor_error, lower_motor_error,
 latent(3 基 × 3 轴), integral_roll, integral_pitch, integral_yaw_rate]
```

- 电机状态与标称版相同：ESC 反馈新鲜且两路同时可用时使用
  `|rpm| * rpm_to_rad_s - trim`，否则回退命令驱动一阶观察器。
- 9 个 latent 状态是 3 个基时间常数（15/40/80 ms）× 3 轴的一阶滤波，输入矩阵为
  `mode_transform @ diag(servo_slopes)`，由三路舵机最终（限幅后）命令驱动；
  复合辨识系数通过增益矩阵耦合到角加速度状态。
- 参考系、四元数、摇杆适配、上下桨差动限幅、饱和冻结积分等约定与标称版一致。
- 新 ABI 位于 `LqiCompositeControllerModel.hpp`，算法位于
  `LqiCompositeCore.hpp`，数值载荷由
  `Identification/datasets/real_logs_26_10_26_v4/offline_identification_v7_hybrid_basic_use.json`
  经生成器写入 `LqiIdentifiedModel.hpp`（禁止手改）。
- 2026-10-26 初版基本可用标定载荷的 model checksum 为 `0x653927a7`，权重字节数为
  580；10 点电机表在控制器命令域中已经吸收最终 `0.5` 输出映射。日志特征质量门控
  已通过。初版 `p/q=0.2` 在实机上仍出现约 5–7 Hz 极限环，因此降振版把 p/q
  状态尺度放宽到 `0.3`，当前 model checksum 为 `0x1716d847`。
- 日志采集时 `MOT_POLE_COUNT=14`，但当前 28 磁极电机应使用 28；启动脚本因此
  强制设置 `MOT_POLE_COUNT=28`。必须核对实体电机磁极数，不能在不重新标定电机表
  的情况下单独修改磁极数或最终输出比例。
- 该载荷通过了黄金向量、4/8 s 四方向极性和反对称检查，可作为低空受控基本悬停
  候选；仍需台架、HIL 和实机 shadow 验证，不能把编译通过解释为飞行授权。

生成器现在同时输出标称和辨识两套 golden 向量；主机检查
`px4_trans/tests/run_lqi_core_check.sh` 会依次验证两个核心。SimEnv 闭环极性
检查脚本为 `Identification/scripts/polarity_check_identified_lqi.py`，结果为
`Identification/datasets/real_logs_26_10_26_v4/polarity_check_basic_use_8s.json`
（四个方向初始角加速度方向正确、4 s 内回正，舵机指令对 ±roll 反对称，输出有界）。

## 2026-08-28 调参：增大积分项与打杆指令比例

针对 `lqi_identified` 真机悬停稳态误差偏大、手动打杆响应不足的问题，保持
辨识数据和编码器不变，做了两处调整：

- **积分项约翻倍**：离线辨识重新合成增益时把 LQR 积分权重提高 4 倍
  （`integral_state_scales` 由 `[0.04, 0.04, 0.25]` 减半为
  `[0.02, 0.02, 0.125]`），新的 `analysis_gain` 中三路积分列约为原来的 2 倍，
  其余列由 LQR 耦合略增。合成配置在
  `Controller/configs/lqi_sim2real_micro_coaxial_4out_strong_integral.yaml`，
  辨识实验配置在
  `Identification/configs/lqr_sim2real_micro_offline_logs_v4_strong_integral.yaml`；
  标称 13 状态 4out 配置与权重保持不变。
- **打杆指令比例提高**：`configs/lqi_nominal_deployment.yaml` 的
  `manual_reference` 由 roll/pitch 12°、yaw 1.2 rad/s 调整为 roll/pitch 18°、
  yaw 1.5 rad/s；滤波时间常数不变。该常量为标称/辨识两版共用的摇杆参考契约，
  标称增益本身未变。

重新生成后的验证：两套主机 golden 检查全部通过；SimEnv 闭环 8 s 扰动回正中
roll 残差 0.16°→0.13°、pitch 残差 0.08°→0.003°；打杆阶跃 t90 由约 0.44 s
缩短到约 0.39 s，四个方向极性检查全部正确、输出有界。辨识模型极点半径由
0.9956 改善到 0.9912。
