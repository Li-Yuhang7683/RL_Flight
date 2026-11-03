# 两机侧拼组合体协同控制器设计规范

状态：`design_only`

日期：2026-11-02

适用对象：两架同型共轴反桨矢量推进无人机沿机体 Y 轴刚性连接后的组合体

## 1. 目的与设计结论

本文设计组合体协同控制器的状态接口、四个虚拟控制通道、执行器分配和理论验证方法。
目标是在不增加双机辨识实验的前提下，复用单体控制器已有的实测和离线辨识参数，
建立一套可用同一组合体反馈驱动的 `C_PAIR` 控制器。

核心设计如下：

1. 两机沿组合体 Y 轴左右排列。主机 A 位于 `(0,+a,0)`，从机 B 位于
   `(0,-a,0)`，其中 `a>0` 是两机标称中心到组合体中心原点 C 的距离。
2. 主机和从机共享组合体姿态、角速度和竖直加速度反馈。每侧不运行独立的姿态闭环。
3. 总距通道复用单体 Z 轴加速度 PI 控制，只负责组合体总升力预算。
4. Roll 通道由左右单体的推力差产生 `Mx`；左右目标推力之和始终等于总距通道给出的预算。
5. Yaw 通道产生统一的组合体 `Mz` 需求，再分解为左右单体上下桨反扭矩；每个单体
   在求解 Yaw 时优先保持其已有目标推力。
6. Pitch 通道复用单体 Pitch 控制参数和原有 Roll/Pitch 格栅分配参数。沿 Y 轴拼接
   在 `Iyy` 上没有额外平行轴项，两个单体共同提供 Pitch 力矩，因此相同命令域的
   单位姿态响应可保持与单体一致。
7. Pitch 指令在组合体系 C 中生成，再通过静态安装拓扑 `T_CA`、`T_CB` 映射到两机
   局部坐标，最后由各自的三个格栅生成特定方向矢量。两机格栅在组合体系中的
   力矩必须同向合成；由于安装方向可能不同，两侧舵机数值不必逐通道相同。

本文是控制系统设计输入，不是已经实现的控制器，也不构成 HIL、台架、系留或实飞
许可。组合姿态观测器、控制路由和 AIRLINK 执行器所有权由各自文档负责。

## 2. 设计范围与约束

### 2.1 本文覆盖

- 组合体姿态角和角速度闭环；
- 归一化总距输入到竖直加速度闭环；
- `[总推力 T_C, Mx, My, Mz]` 四个虚拟控制量；
- 左右五路物理命令的协同分配；
- 静态安装拓扑及其配置接口；
- 饱和、积分冻结、观测失效和拓扑失效处理；
- 仅使用现有单体参数和模型的理论验证计划。

### 2.2 本文不覆盖

- 组合体姿态估计算法、双 IMU 时间同步和组合 EKF；
- 单机/组合控制路径的最终 owner 仲裁；
- AIRLINK v1/v2 消息编码和路由状态机；
- 水平位置、速度、轨迹或气动互扰补偿；
- 对接面的结构强度、螺栓预载、滑移、柔性和疲劳；
- 新双机风洞、系统辨识、HIL、无桨台架、系留和实飞试验。

### 2.3 首版包线

首版仅声明悬停、低速和小中倾角下的设计适用性。控制器使用静态拓扑和现有单体
执行器模型，不对以下情况作性能承诺：

- 两机之间的旋翼尾流、格栅射流和气动遮蔽；
- 对接结构柔性导致两侧姿态或加速度不再严格共刚体；
- 大倾角或中高速前飞导致有效推力轴和操纵效率显著变化；
- 非刚性连接、连接件回差或运行中拓扑变化。

## 3. 坐标系、符号与几何

### 3.1 坐标系

- `N`：公共 NED 世界系；
- `A`：主机 FRD 本体系，`+x` 前、`+y` 右、`+z` 下；
- `B`：从机 FRD 本体系；
- `C`：组合体 FRD 虚体系，原点位于组合体标称重心。

C 拥有独立 FRD 轴向：主机前向轴绕主机 +Z（向下）右转 30°得到 C 前向轴；
从机前向轴绕从机 +Z 左转 30°得到同一 C 前向轴。两机前向轴夹角 60°，C 在中间。
向量分量的标定映射为 `R_CA=Rz_FRD(-30°)`、`R_CB=Rz_FRD(+30°)`；不得令 `R_CA=I`。
一般装配使用标定的完整旋转矩阵 `R_CA`、`R_CB`，yaw 参数表示当前共面装配的特例。

四元数统一使用 Hamilton `w,x,y,z` 顺序，`q_目标源` 将源系向量映射到目标系：

- `q_NC`：C 系到 NED；
- `q_NC_ref`：目标 C 系到 NED；
- `q_AB`：B 系到 A 系的静态安装旋转，`q_AB=q_CA^(-1)⊗q_CB`；
- `q` 与 `-q` 等价，姿态差必须使用最短旋转。

### 3.2 两机位置

在 C 系中：

```text
r_A = [0, +a, 0]
r_B = [0, -a, 0]
```

两侧仍可各自绕共同推力轴有静态安装旋转。`a`、每个单体的方向和安装拓扑都必须
来自同一版本化配置，不能由控制器运行时猜测。

### 3.3 静态安装拓扑

控制器使用一个只读的 `InstallationTopology` 逻辑接口：

| 字段 | 语义 |
| --- | --- |
| `version` | 拓扑格式版本 |
| `R_CA[9]` | A 系到 C 系的静态旋转矩阵 |
| `R_CB[9]` | B 系到 C 系的静态旋转矩阵 |
| `r_A_C[3]` | A 系原点相对 C 系原点的位置，表达在 C 系 |
| `r_B_C[3]` | B 系原点相对 C 系原点的位置，表达在 C 系 |
| `a_m` | 两机中心到 C 系原点的标称距离 |
| `mass_a_kg`、`mass_b_kg` | 两机质量 |
| `connector_mass_kg` | 对接结构质量，未知时为零并明确标为不完整 |
| `connector_inertia_kg_m2[3]` | 对接结构在 C 系中的惯量 |
| `fingerprint` | 几何、安装、参数和协议兼容性的联合哈希 |
| `valid` | 拓扑已标定、有限、正交且版本兼容 |

`R_CA`、`R_CB` 必须满足：

```text
R R^T = I,  det(R) = +1
R_CI = R_IC^T
```

向量和转动惯量的变换为：

```text
v_C = R_CI v_i
v_i = R_CI^T v_C
I_i_C = R_CI I_i R_CI^T
```

任意位置的变换为：

```text
p_C = r_i_C + R_CI p_i
```

该接口只描述已经标定的静态安装关系，不负责估计安装角。控制器和组合姿态观测器
必须使用兼容的拓扑指纹；任一方的拓扑版本或指纹不一致时，组合控制器不得进入
有效控制状态。

### 3.4 力矩和合力约定

单体力/力矩合成到 C 系：

```text
F_C = sum_i(R_CI F_i)
M_C = sum_i[R_CI M_i + r_i_C x (R_CI F_i)]
```

所有力矩按右手定则定义。符号验证必须至少覆盖：

- `Mx>0` 时，右机 B 应比左机 A 提供更大的向上推力；
- `My>0` 时，两机的格栅力矩在 C 系应同向合成；
- `Mz>0` 时，两机上下桨反扭矩在 C 系应同向合成；
- 安装旋转后，先合成到 C 系再判断方向，禁止直接比较两机局部舵机数值的正负。

## 4. 控制架构

```text
CompositeAttitudeEstimate
  q_NC, omega_C, linear_acceleration_n, health
                         |
                         v
              +------------------------+
              | 统一组合体姿态与速率   |
              +------------------------+
                  |       |        |
          Roll LQI | Pitch | Yaw LQI | Collective Z-acceleration PI
                  v       v        v                 v
               Mx_C     My_C     Mz_C              T_C
                  |       |        |                 |
                  +-------+--------+-----------------+
                                  |
                      +--------------------------+
                      | 组合体执行器分配器       |
                      | Roll -> 总推力差         |
                      | Pitch -> 格栅拓扑映射    |
                      | Yaw -> 上下桨反扭矩      |
                      +--------------------------+
                                  |
                    +-------------+-------------+
                    v                           v
             A: [uU,uL,s1,s2,s3]       B: [uU,uL,s1,s2,s3]
```

物理命令沿用现有 AIRLINK 五路执行器契约：

```text
[upper_motor, lower_motor, servo_1, servo_2, servo_3]
```

主机上的 `C_PAIR` 控制器一次产生 A、B 两套五路命令。从机只执行主机下发的新鲜命令，
不自行运行第二套组合姿态闭环。该输出可以映射到现有
`AirlinkPairCommand.master_*` 和 `AirlinkPairCommand.follower_*` 接口。

## 5. 逻辑数据接口

### 5.1 `CompositeControlState`

| 字段 | 语义 |
| --- | --- |
| `publish_time_us` | 状态发布时刻 |
| `sample_time_us` | 公共 IMU 采样时刻 |
| `q_NC[4]` | C 系到 NED，Hamilton WXYZ |
| `omega_C[3]` | C 系角速度，rad/s |
| `linear_acceleration_n[3]` | NED 中的运动学线加速度，m/s²，不是比力 |
| `yaw_sigma_rad` | 组合 yaw 锚点不确定度，仅诊断 |
| `health_flags` | 观测器健康位 |
| `estimator_epoch` | 组合估计会话 |
| `topology_fingerprint` | 观测器使用的安装拓扑指纹 |
| `valid` | 是否允许控制计算 |

如果观测器只能提供虚拟 IMU 比力 `f_C`，适配层必须先按重力、姿态和杠杆关系转换为
NED 运动学加速度。控制器不得把静止悬停时的 `f_C,z≈-g` 直接当作下降加速度。

### 5.2 `CompositeControlReference`

| 字段 | 语义 |
| --- | --- |
| `q_NC_ref[4]` | 目标组合体姿态 |
| `omega_C_ref[3]` | 目标组合体角速度，rad/s |
| `collective[0,1]` | 归一化总距/竖直加速度命令 |
| `mode` | 组合控制模式，首版仅 `ATTITUDE_ACCELERATION` |

目标姿态和速率来自已有飞手/虚拟飞手参考，不在本控制器中实现位置闭环。

### 5.3 `CompositeActuatorCommand`

| 字段 | 语义 |
| --- | --- |
| `timestamp_sample` | 本次控制所用状态采样时刻 |
| `valid` | 两套命令是否完整、有限且可交给 owner |
| `unit_a[5]` | A 的 `[uU,uL,s1,s2,s3]` |
| `unit_b[5]` | B 的 `[uU,uL,s1,s2,s3]` |
| `virtual_command[4]` | `[T_C,Mx,My,Mz]` 请求 |
| `achieved_wrench[4]` | 模型估算的实际 `[T_C,Mx,My,Mz]` |
| `diagnostics` | 饱和、映射残差和积分状态 |

`valid=false` 时，owner 不得使用同一消息中的旧命令继续控制。平台级 failsafe 和
owner 路由不由控制器改变。

### 5.4 `GridTopologyMap`

Pitch 通道需要独立的静态映射接口：

```text
map_grid(
    topology,
    unit_id,
    target_pitch_vector_C,
    total_thrust_target,
    trim_command
) -> (
    servo_command[3],
    achieved_moment_C[3],
    thrust_residual_C,
    direction_scale,
    valid,
    fault_reason
)
```

该接口封装以下职责：

1. 将 C 系目标 Pitch 方向变换到对应单体坐标；
2. 调用单体已有的 Roll/Pitch 格栅控制参数和三舵机分配参数；
3. 返回该单体在 C 系中实际合成的力矩；
4. 报告格栅造成的推力、Roll 和 Yaw 残差；
5. 在舵机限幅时对完整方向矢量做统一缩放，而不是逐舵机截断。

接口名称是实现建议，不要求引入新的线上消息。拓扑数据可以来自部署配置、参数表或
manifest，但必须是只读、版本化且可记录指纹的。

## 6. 组合体刚体模型

### 6.1 质量和惯量

组合体重心满足：

```text
M_C = m_A + m_B + m_S
r_C = (m_A r_A + m_B r_B + m_S r_S) / M_C
I_C = sum_i {
    R_CI I_i R_CI^T
    + m_i [ (r_i-r_C)^T(r_i-r_C) I - (r_i-r_C)(r_i-r_C)^T ]
} + I_S_C
```

若两机等质量、等惯量、同向且关于 C 原点对称，忽略连接件，则：

```text
Ixx_C = 2 Ixx_unit + 2 m a^2
Iyy_C = 2 Iyy_unit
Izz_C = 2 Izz_unit + 2 m a^2
```

`Iyy_C` 不包含 `m a^2` 平行轴项。组合体 Pitch 的总惯量是两个单体自身 Pitch
惯量之和，而左右单体共同产生 Pitch 力矩，因此同一归一化姿态命令下的角加速度
关系与单体一致。这是复用单体 Pitch 控制参数范围的物理依据。

以当前单体基线 `m=0.66 kg`、`I_unit=diag(5.0e-4,5.0e-4,7.333333e-4) kg m²`
和 `a=0.075 m` 计算，不含对接结构时：

| 项目 | 数值 |
| --- | ---: |
| 组合体质量 | `1.320 kg` |
| `Ixx_C` | `8.425e-3 kg m²` |
| `Iyy_C` | `1.000e-3 kg m²` |
| `Izz_C` | `8.892e-3 kg m²` |

这些数值只用于设计分析。实际对接质量、连接件、线束和装配偏移必须加入配置；
没有完整几何指纹时不得把上表当作最终参数。

### 6.2 单片执行器模型

单体继续使用
`SimEnv/configs/sim2real_micro_coaxial_3100kv_airframe1_manual_v2.yaml`
中的电机表、推力系数、反扭矩系数、舵机表和格栅模型。设单体上、下桨指令为
`(uU,uL)`，三个格栅指令为 `s=[s1,s2,s3]`，现有模型给出：

```text
T_i = T_u(uU) + T_l(uL) + T_cross(uU,uL)
Mz_i = -(Q_u(uU) - Q_l(uL))
```

真实方向由配置的推力轴和反扭矩轴决定；实现必须沿用现有 `LocalPlantModel` 或
其等价只读映射，不得在控制器里另写一套未经验证的系数。

### 6.3 垂直轴方向

首版标称拓扑假设两机推力主轴在 C 系中同向并沿 `-z`。允许两机绕该共同轴有静态
安装旋转，因此该旋转不改变总升力方向，但会改变格栅产生的局部 Roll/Pitch 分量。

若拓扑接口返回的两机推力轴不平行，完整的力/力矩混合属于后台自由度分配问题，
必须另行验证。不得仅使用本节对称公式后宣称任意三维安装角都可安全控制。

## 7. 姿态与竖直加速度控制

### 7.1 姿态误差

定义最短姿态误差：

```text
dq = q_NC_ref^(-1) * q_NC
if dq.w < 0: dq = -dq
e_att = 2 atan2(||dq.xyz||, dq.w) * dq.xyz / ||dq.xyz||
e_omega = omega_C - omega_C_ref
```

当 `||dq.xyz||` 接近零时使用单位矩阵极限，不能除以零。Roll、Pitch、Yaw 的正负
必须通过固定初值 Golden 测试与 SimEnv 的 FRD 力矩方向逐一核对。

### 7.2 总距通道

总距通道复用现役单体
`CoaxialCollectiveServoController` 的“飞手总距 -> 竖直加速度参考 -> PI”结构。
首轮复用
`Controller/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml`
中的以下参数：

| 参数 | 首轮值 |
| --- | ---: |
| `maximum_acceleration_m_s2` | `5.0` |
| `acceleration_kp` | `1.0` |
| `acceleration_ki` | `4.0` |
| `acceleration_integral_limit_m_s2` | `2.0` |
| `acceleration_filter_time_constant_s` | `0.010` |
| `minimum_thrust_fraction` | `0.02` |
| `maximum_thrust_fraction` | `0.98` |
| `zero_throttle_threshold` | `0.02` |

设 NED 运动学加速度为 `a_N`，向上加速度为：

```text
a_up_measured = -a_N.z
```

一阶低通：

```text
alpha = dt / (tau + dt)
a_up_filtered[k] = a_up_filtered[k-1]
                 + alpha (a_up_measured[k] - a_up_filtered[k-1])
```

飞手归一化总距 `c` 映射到目标加速度 `a_ref`，计算：

```text
e_a = a_ref - a_up_filtered
I_a = clamp(I_a_previous + e_a dt, -I_max, +I_max)
a_corrected = a_ref + Kp_a e_a + Ki_a I_a
T_C = M_C (g + a_corrected) / max(cos_tilt, cos_min)
```

其中 `cos_tilt` 是 C 系 `-z` 轴与世界天向的夹角余弦。总推力按两机标称总推力范围
限幅。零总距时两机电机命令置零，并清除总距积分。

### 7.3 三轴分轴 LQI

Roll、Pitch、Yaw 使用三个独立 LQI：

```text
x_roll  = [e_att.x, p-p_ref,     integral(e_att.x)]
x_pitch = [e_att.y, q-q_ref,     integral(e_att.y)]
x_yaw   = [e_att.z, r-r_ref,     integral(r-r_ref)]

u_j = -K_j x_j
[u_roll,u_pitch,u_yaw] = [Mx,My,Mz]
```

Yaw 的积分项使用角速度误差，避免在无独立组合航向观测时长期积分共同 yaw 漂移。
当组合航向绝对基准可用后，是否改为角度误差积分必须通过独立设计评审决定。

单轴连续模型为：

```text
angle_dot = rate
rate_dot  = moment / I_axis
```

以 500 Hz 对“双积分 + 积分状态”做 ZOH 离散，并求解离散代数 Riccati 方程：

```text
Q = diag(1/angle_scale^2, 1/rate_scale^2, 1/integral_scale^2)
R = input_weight_scale / moment_scale^2
K = (R + Bd^T P Bd)^(-1) Bd^T P Ad
```

首轮角度、速率和力矩尺度沿用单体 LQI 参数：

| 轴 | 角度尺度 | 速率尺度 | 积分尺度 | 力矩尺度 |
| --- | ---: | ---: | ---: | ---: |
| Roll | `10 deg` | `0.7 rad/s` | `0.1 rad` | `0.008 N m` |
| Pitch | `10 deg` | `0.7 rad/s` | `0.1 rad` | `0.008 N m` |
| Yaw | `10 deg` | `1.5 rad/s` | `0.4 rad/s` | `0.020 N m` |

Roll 和 Yaw 的 `I_axis` 使用组合体 `Ixx_C`、`Izz_C`；Pitch 使用单体控制器的命令域
参数，详见第 8.3 节。

### 7.4 积分和抗饱和

出现以下任一条件时冻结对应积分累加：

- 通道达到力矩权限；
- 执行器达到对应约束；
- 观测器无效或拓扑无效；
- 控制器不是当前 active owner；
- 零总距；
- 系统正在重新建立拓扑或估计 epoch。

冻结时不得继续累加误差，也不得用简单清零代替抗饱和。恢复 active 后是否清空积分
由路由切换规则决定；新 `C_PAIR` 会话必须从已验证初值开始，不能继承过期单体积分。

## 8. 四通道执行器分配

### 8.1 分配顺序

分配器按固定顺序执行，以便逐级施加守恒约束：

```text
1. T_C -> 左右基准推力 T_A0/T_B0
2. Mx  -> Roll 推力差，得到 T_A/T_B，保持 T_A+T_B
3. My  -> 安装拓扑与单体格栅分配，保持 Pitch 方向
4. Mz  -> 每个单体上下桨反扭矩，在各自 T_i 目标附近求解
5. 限幅、残差记录和诊断
```

不能用四个通道分别直接修改同一路电机或舵机后再逐路裁剪。每次裁剪都必须回到
该通道定义的守恒条件重新求解。

### 8.2 总距与 Roll

设总距给出组合体目标推力 `T_C`。等质量对称安装时：

```text
T_A0 = T_B0 = T_C / 2
```

若配置给出已知的静态配平偏移，可在不改变总和的情况下加入
`T_A0`、`T_B0` 配平项。

两机位置满足 `r_A.y=+a`、`r_B.y=-a`，向上推力为负 Z 时：

```text
Mx = a (T_B - T_A)
```

因此 Roll 需求对应的推力差为：

```text
delta_T = Mx / a
T_B = T_C/2 + delta_T/2 = T_C/2 + Mx/(2a)
T_A = T_C/2 - delta_T/2 = T_C/2 - Mx/(2a)
```

在没有饱和时：

```text
T_A + T_B = T_C
```

这是 Roll 通道的硬约束。目标推力必须先转换回单体指令域：

```text
c_A = T_to_common_command(T_A, unit_parameters_A)
c_B = T_to_common_command(T_B, unit_parameters_B)
```

其中 `T_to_common_command` 使用单体上下桨同向时的推力映射和现有转速表反解。
禁止直接把 `Mx` 平均加到两个电机通道，因为不同工作点的推力斜率不同，会破坏总推力。

在对称单机推力上下限 `[T_min,T_max]` 下，可用 Roll 力矩为：

```text
Mx_limit_neg = -a min(2 T_C/2 - 2 T_min, 2 T_max - 2 T_C/2)
Mx_limit_pos = +a min(2 T_C/2 - 2 T_min, 2 T_max - 2 T_C/2)
```

实际实现还须加入电机死区、上下桨差动限制和单体健康裕量。若 `Mx` 超过可达到值，
保持 `T_A+T_B=T_C`，只降低 `Mx`；不得提高或降低总距来换取 Roll。

### 8.3 Pitch、参数复用与安装拓扑映射

#### 8.3.1 为什么可以复用

两机沿 Y 轴排列只给 `Ixx_C` 和 `Izz_C` 增加平行轴惯量，不给 `Iyy_C` 增加 `m a²`
项。组合体 Pitch 的总惯量为：

```text
Iyy_C = 2 Iyy_unit + Iyy_connector
```

在同一 C 系 Pitch 方向上，左右单体都产生同向格栅力矩，总 Pitch 力矩约为左右单体
各自力矩之和。等质量、等惯量、两机均未饱和时：

```text
My_pair / Iyy_pair
    ≈ (2 My_unit) / (2 Iyy_unit)
    = My_unit / Iyy_unit
```

因此 Pitch 外环继续使用单体 Pitch 参数，不针对 `a` 重新设计新的 Pitch 惯量增益。
连接件惯量会打破这一等比例关系，必须在没有连接件参数时把该结论标为“不完整模型”；
不能声称任意连接件质量下仍严格复用。

复用范围：

- 单体 Pitch 外环的误差、角速度和积分参数；
- 单体 Roll/Pitch 格栅分配参数；
- 单体三舵机映射、正则化、自衰减和限幅参数；
- 现有单体执行器工作点附近的模型。

不直接复用单体 Roll 外环增益，因为组合体绕 X 轴的 `Ixx_C` 明显增加，且本方案的
Roll 主要由左右推力差产生。

#### 8.3.2 Pitch 命令到局部坐标

设复用单体 Pitch 控制参数后，在 C 系得到虚拟格栅矢量：

```text
v_pitch_C = [vx_C, vy_C, vz_C]
```

对本方案，主要分量是 `vy_C`，但接口保留三维表示，以便安装旋转或后续耦合补偿使用。
两机在 C 系的目标份额为：

```text
v_A_C = w_A v_pitch_C
v_B_C = w_B v_pitch_C
w_A + w_B = 1
```

标称等质量时 `w_A=w_B=0.5`。每机局部目标为：

```text
v_A_local = R_CA^T v_A_C
v_B_local = R_CB^T v_B_C
```

再把 `v_i_local` 交给该单体现有的三格栅分配器。若 B 相对 C 有安装 yaw 旋转，
C 系纯 Pitch 在 B 局部会分解为 Roll/Pitch 混合；对应三个舵机命令会不同。这是预期
的参考系映射，不是两侧控制反馈不同。

“两侧同步格栅”定义为：

- 两侧使用同一组合体 C 系 Pitch 反馈和同一控制时刻；
- 两侧在 C 系产生的目标力矩同向；
- 两侧目标矢量按耦合矩阵合成后与请求方向一致。

它不要求 `s_A.xyz == s_B.xyz`。只有当 `R_CB=I` 且单体参数相同时，舵机命令才可能相同。

#### 8.3.3 三舵机方向映射

对每个单体建立只读的局部正向映射：

```text
w_i_local = GridWrench_i(s_i, T_i, trim_i)
```

其中 `w_i_local` 至少包含局部 `Mx/My/Mz` 和推力残差。现有单体分配器负责求逆：

```text
s_i_requested = GridInverse_i(v_i_local, T_i)
```

拓扑层把每机局部结果变换到 C 系并求和：

```text
w_A_C = R_CA w_A_local
w_B_C = R_CB w_B_local
w_pair_C = w_A_C + w_B_C + r_A x F_A_C + r_B x F_B_C
```

Pitch 方向验收条件为 `w_pair_C` 的 `My` 分量与请求同号，且非 Pitch 分量的残差在
配置门限内。映射接口必须返回这些残差，不能只返回“命令已发送”。

Pitch 分配过程中：

- 不直接修改四电机主要分配；
- 不因单个舵机先饱和而逐路裁剪；
- 若两机联合需要缩放，使用同一个 `alpha in [0,1]` 缩放两侧目标矢量；
- 缩放导致 Pitch 力矩不足时冻结 Pitch 积分；
- 格栅产生的推力、Roll、Yaw 残差必须记录，供总距慢环和 Yaw 分配处理。

Pitch 控制器对电机分配保持功能独立；格栅改变总推力属于执行器物理耦合，不意味着
Pitch 通道可以直接改写电机基线。

### 8.4 Yaw 与单机推力保持

Yaw 控制器给出组合体 `Mz_C`。首轮按两机可用能力分配：

```text
w_i = k_yaw_i / (k_yaw_A + k_yaw_B)
Mz_A = w_A Mz_C
Mz_B = w_B Mz_C
```

等质量、同参数时 `Mz_A=Mz_B=Mz_C/2`。

如果 Pitch 格栅在 C 系产生估计的 `Mz_grid`，电机 Yaw 分配只承担：

```text
Mz_motor = Mz_C - Mz_grid
```

对每个单体，在 Roll 阶段已经确定的 `T_A`、`T_B` 附近，求解上下桨命令：

```text
find (uU_i,uL_i)
subject to:
    0 <= uU_i <= 1
    0 <= uL_i <= 1
    |uU_i-uL_i| <= differential_limit
priority 1: T_i(uU_i,uL_i) = T_i_target
priority 2: Mz_i(uU_i,uL_i) = Mz_i_target
priority 3: minimize command deviation from c_i
```

这是一个“先保单机推力、再满足反扭矩、最后选最小命令偏移”的字典序问题。实现可使用
基于现有单体模型预计算的二维插值表、局部 Newton 解或等推力曲线搜索，但不得为了
得到 Yaw 而重新定义电机推力模型。

在两侧电机表中，上、下桨同工作点的反扭矩单元可以近似线性相消，因此围绕配平点
的小差动可产生 Yaw 而几乎不改变推力。但以下非线性必须处理：

- 下桨工作点两侧斜率不同；
- 上、下电机一阶时间常数不同；
- 电机死区和 PWM 量化；
- 接近满量程时可用差动减小；
- 推力与反扭矩因转速二次关系而产生二阶耦合。

若单体在保持 `T_i_target` 时无法达到请求 `Mz_i`，先减少该侧 Yaw 需求，再把剩余
需求转移给另一侧。两侧都饱和时按比例降低总 `Mz`，并冻结 Yaw 积分。禁止通过改变
`T_A`、`T_B` 偷取 Yaw 权限，否则 Roll 总推力守恒被破坏。

### 8.5 分配结果和残差

最终输出为：

```text
unit_A = [uU_A, uL_A, s1_A, s2_A, s3_A]
unit_B = [uU_B, uL_B, s1_B, s2_B, s3_B]
```

分配器必须同时报告：

- `T_C_requested`、`T_C_achieved`；
- `T_A_target/achieved`、`T_B_target/achieved`；
- `Mx_requested/achieved`；
- `My_requested/achieved`；
- `Mz_requested/achieved`；
- 每轴饱和比例；
- 拓扑映射后的局部目标矢量；
- 格栅造成的推力、Roll、Yaw 残差；
- Yaw 因单机推力约束而被削减的力矩。

在模型检查中，未饱和时以下条件必须满足：

```text
T_A_achieved + T_B_achieved = T_C_requested
|T_i_achieved - T_i_target| <= thrust_tolerance
Pitch motor commands unchanged by Pitch allocation
```

有饱和或未建模气动干扰时，输出必须标记降级，不能继续报告“守恒成立”。

## 9. 控制生命周期与失效处理

### 9.1 启动前置条件

进入组合控制候选有效状态前，必须同时满足：

- `CompositeControlState.valid=true`；
- 观测器 `estimator_epoch` 已稳定；
- 观测拓扑指纹与控制器拓扑指纹一致；
- 拓扑矩阵有限、正交且 `det(R)=+1`；
- 单体控制器配置、电机表、舵机表和格栅参数版本一致；
- AIRLINK 会话和 owner 允许组合控制；
- 两机实际安装方向与配置一致；
- 控制器完成有限性和初值检查。

不满足时不得使用旧状态或旧命令继续计算。

### 9.2 500 Hz 执行

控制节拍固定为 `dt=2 ms`。每次执行：

1. 验证状态、参考和拓扑；
2. 计算统一姿态/加速度反馈；
3. 计算 `[T_C,Mx,My,Mz]`；
4. 按第 8 节顺序完成约束分配；
5. 检查两套五路命令的有限性和范围；
6. 发布命令及诊断。

不得在组合和单体 owner 之间按轴混合输出。控制切换只发生在固定节拍边界，具体规则见
[AIRLINK 协议](AIRLINK_PROTOCOL_V1_zh.md) 和组合姿态估计器文档。

### 9.3 失效条件

以下情况使 `CompositeActuatorCommand.valid=false`：

- 组合状态超过允许的新鲜度；
- `sample_time_us` 不前进或发生不兼容回绕；
- `estimator_epoch` 改变；
- 姿态四元数、角速度、加速度或参考含 NaN/Inf；
- 拓扑指纹改变；
- 旋转矩阵非正交或安装拓扑越界；
- 目标姿态或总距超出协商范围；
- 输出命令含 NaN/Inf；
- 控制器尚未完成本次 `C_PAIR` 会话初始化。

失效后的立即动作：

- 冻结全部积分；
- 禁止发布新的有效候选命令；
- 不复用最后一帧旧命令；
- 记录首个故障原因和 epoch；
- 由 owner/failsafe 决定停机、降落或显式切换，不自动改为 `SINGLE`。

恢复时必须重新满足启动前置条件，并以新控制器会话清空或显式重建内部状态。

## 10. 参数、版本和可观测性

### 10.1 参数来源

首轮参数来源固定为以下单体数据和已有报告：

- 单体物理模型：
  `SimEnv/configs/sim2real_micro_coaxial_3100kv_airframe1_manual_v2.yaml`；
- 单体总距/姿态控制：
  `Controller/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml`；
- 双机几何和初步权限：
  [双机体刚性拼接协同控制可行性烟测报告](../Identification/PAIR_ASSEMBLY_FEASIBILITY_zh.md)；
- 对接面约束与载荷：
  [双机体对接面载荷与最小载荷控制模态](../Identification/PAIR_INTERFACE_LOAD_zh.md)。

这些文件中的参数来源和实验边界各有不同。引用时必须记录文件内容哈希，不能只记录
文件名；双机辨识缺失不会被本设计掩盖。

### 10.2 最小日志字段

每个组合控制日志至少记录：

- `sample_time_us`、控制时刻、`estimator_epoch`、`route_epoch`；
- `q_NC`、`omega_C`、`linear_acceleration_n`；
- 目标姿态、目标速率和飞手总距；
- 姿态误差、各轴积分器；
- `[T_C,Mx,My,Mz]` 请求和模型估算值；
- A/B 两套五路命令；
- `r_A/r_B`、`R_CA/R_CB`、拓扑指纹；
- `v_pitch_C`、两侧局部 Pitch 目标矢量、三舵机命令；
- 各单体目标/实际推力和反扭矩；
- 总推力、Roll、Yaw 守恒残差；
- 每个通道的限幅、缩放和积分冻结；
- 控制总耗时和状态年龄。

日志必须能够区分：输入陈旧、拓扑映射错误、执行器饱和、单体模型误差和气动干扰。

## 11. 理论验证计划

本节只定义设计阶段可用的数学和模型验证，不代表已经完成的测试。

### 11.1 几何与符号

- 在 `r_A=(0,+a,0)`、`r_B=(0,-a,0)` 下验证
  `Iyy_C=2 Iyy_unit+Iyy_connector`，并验证 `Ixx_C`、`Izz_C` 的平行轴项；
- 对齐安装、B 相对 A 旋转 90/180 度，以及实际安装拓扑三种情况；
- 验证 `R R^T=I`、`det(R)=+1`、平移和旋转逆变换；
- 验证正向 Roll、Pitch、Yaw 在 C 系中的力矩符号；
- 验证 `q` 与 `-q` 产生同一姿态误差，90/180 度不出现欧拉奇异。

### 11.2 Pitch 参数复用和拓扑映射

- 恒等拓扑下，两侧相同 Pitch 方向产生与单体相同的单位惯量角加速度；
- B 绕共同推力轴旋转后，B 局部三舵机方向矢量应与旋转一致，而 C 系合成方向仍为正 Pitch；
- 改变 `T_CA/T_CB` 时，只允许改变方向映射和物理分配，不得静默改变单体 Pitch 参数；
- 当连接件惯量加入后，报告 `Iyy_C` 偏离单体两倍的程度，并验证复用结论是否仍成立；
- 一侧航向安装参数错误时，C 系 Pitch 合成残差应超过门限并使输出无效。

### 11.3 守恒和分配

- 任意 `T_C` 和未饱和 `Mx` 下验证
  `T_A_achieved+T_B_achieved=T_C`；
- Roll 饱和时验证总推力仍保持，只有 `Mx` 被降低；
- Yaw 求解前后验证每个单体推力误差不超过设计容差；
- Yaw 一侧饱和时验证剩余请求转交另一侧，并验证最终 `Mz` 报告正确；
- Pitch 分配不得直接改变四电机命令；
- 格栅方向饱和时验证完整舵机方向矢量统一缩放，不能出现单舵机先裁剪造成的方向偏转。

### 11.4 闭环模型验证

使用现有单体非线性和由静态拓扑合成的组合体刚体模型，以 500 Hz 运行：

- 零姿态误差、零角速度和悬停总距；
- Roll、Pitch、Yaw 单轴小角度阶跃；
- 三轴同时小指令；
- 左右反向初始 Roll；
- 旋转安装后的 Pitch 阶跃；
- 低总距和接近最大总距下的 Roll 权限；
- Yaw 需求达到单侧和双侧差动极限；
- 2 度倾角、0.1 rad/s 角速度的量级和 0.5 s 驻留判据；
- 60 度安全倾角边界外不宣称可控。

首轮验收至少要求：

- 数学变换和未饱和守恒残差在浮点设计容差内；
- LQI 离散闭环极点半径小于 1；
- 无持续未解释的饱和或极限环；
- Pitch 拓扑映射不会把正 Pitch 请求合成为负 Pitch；
- 观测/拓扑故障能在一个控制节拍内撤销 `valid`，但不能自动改变 owner；
- 日志可以复现每帧虚拟命令到两套五路命令的完整映射。

这些判据是模型级设计门槛，不是飞行验收门槛。

### 11.5 明确不包含的验证

- 双机传感器噪声、时间同步和组合 EKF 精度；
- HIL 与真实 AIRLINK 丢包、延迟和排序；
- 舵机死区、回差、限幅和真实安装矩阵误差下的台架试验；
- 系留偏航权限、接口载荷和结构安全；
- 两机气动互扰、地面效应和真实电池电压下降；
- 整机飞行品质、失速、结构疲劳或应急着陆认证。

上述项目完成前，本文只能用于设计评审和下一步实现准备。

## 12. 主要风险与待验证项

1. **静态拓扑模型与真实结构不一致**：连接件柔性、预紧、回差和线束会破坏刚体假设。
2. **组合 Pitch 参数复用的边界**：只有 `Iyy_C≈2Iyy_unit` 且两侧均未饱和时，
   命令域复用才有直接物理依据；连接件惯量必须纳入模型。
3. **安装旋转误差**：Pitch 到三舵机的方向映射对静态安装角敏感，错误的 `q_AB`
   会产生 Roll/Pitch 交叉耦合。
4. **格栅耦合残差**：单体格栅模型包含自衰减和互耦，Pitch 可能带来总推力、Roll 和
   Yaw 残差；首版必须将其作为诊断和慢环补偿处理，不能假定完全独立。
5. **Yaw 权限有限**：两机侧拼增加了 `Izz_C`，偏航反扭矩只来自两机自身上下桨差动；
   偏航是首版最可能的饱和瓶颈。
6. **接口载荷未进入控制设计**：本文协调净运动，不证明连接件安全。差值推力、偏航
   反扭矩和格栅耦合均可能给对接面施加约束载荷。
7. **没有双机辨识**：所有组合性能结论来自单体参数加几何合成，必须保留模型误差裕度。
8. **航向没有独立绝对观测**：统一组合反馈不等于组合 yaw 可观测；不能把三路姿态
   一致当作独立航向证据。
9. **`C` 系与 A 系的兼容关系**：首版采用 C=A 轴向。若未来修改 C 系定义，控制器
   和观测器必须一起更新拓扑版本和指纹。

## 13. 后续实现前置条件

编码前必须冻结以下输入：

- 实际两机中心位置 `a`、组合重心和连接件质量/惯量；
- `R_CA`、`R_CB` 的 CAD 或地面标定值及其误差；
- 组合状态接口中竖直加速度的语义（运动学加速度或比力）；
- `GridTopologyMap` 的具体部署位置和调用开销；
- 现有单体 Pitch/Roll 参数的版本和哈希；
- 两套五路命令的发布通道、owner 和失败路由；
- 总推力、Roll 差动、Yaw 差动和舵机方向的饱和门限；
- 组合体执行器和对接面的载荷测量方案。

在这些输入冻结前，不允许把本文作为 PX4 参数表、固件实现说明或飞行许可。

## 14. 参考

- [组合体姿态估计器设计规范](COMPOSITE_ATTITUDE_ESTIMATOR_zh.md)
- [AIRLINK v1 协议](AIRLINK_PROTOCOL_V1_zh.md)
- [AIRLINK 固件实现](AIRLINK_FIRMWARE_zh.md)
- [双机体刚性拼接协同控制可行性烟测报告](../Identification/PAIR_ASSEMBLY_FEASIBILITY_zh.md)
- [双机体对接面载荷与最小载荷控制模态](../Identification/PAIR_INTERFACE_LOAD_zh.md)
- `Controller/src/flight_controller/coaxial_collective.py`
- `Controller/src/flight_controller/plant.py`
- `SimEnv/configs/sim2real_micro_coaxial_3100kv_airframe1_manual_v2.yaml`
- `Controller/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml`
