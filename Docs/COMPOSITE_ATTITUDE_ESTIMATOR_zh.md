# 组合体 CEKF 设计要求

## 模块边界

PX4 模块名为 `comp_attitude`，只在主机运行，持有自己的 `Ekf` 实例和 `CEKF_*` 参数。
两机原有 EKF2 独立运行；模块只读取主机姿态、主机 IMU 和 Airlink 从机 IMU。
不修改两机 EKF2 参数，不向 `vehicle_odometry` 注入 yaw。

保留 `composite_attitude_estimate`、`composite_estimator_status`、
`composite_yaw_reference` 和 `mount_orientation` 输出，兼容现有观察与传输端。

## 坐标系

A、B 为两机 FRD 机体系，C 为组合体 FRD 系。
向量映射为 `v_C = Rz(CEKF_*_YAW) v_X`。
默认 A→C 为 -30°，B→C 为 +30°。
主机姿态初始化使用 `q_NC = q_NA * inverse(q_CA)`，包含 roll、pitch、yaw。

## 连接状态与初始化

- 模块启用且主机姿态可用时，可以先用主机姿态初始化并以主机 IMU 更新。
- 每次 Airlink 建链或对端 session 改变，立即重新创建 CEKF，用当前主机姿态旋转至 C 系后初始化。
- 直接设置该独立实例的姿态、协方差和输出预测器，跳过默认静止重力对准；飞行中也能初始化。
- 不等待从机姿态、两机姿态一致性、时钟窗口收敛、100 个样本或额外预热。
- Airlink 连接依据 `airlink_status` 的主机角色、heartbeat/sensor freshness；状态发布超过 100 ms 未刷新也视为断链。
  没有新从机帧的单个调度周期不等于断链。物理断线的检测延迟由 Airlink freshness 和状态发布周期决定。
- 尚无主机姿态或 IMU 时，等待所需输入，不伪造姿态或 IMU。

## IMU 更新

主机和从机均按本地 `sensor_selection` 分别选择加速度计和陀螺仪，
从对应的 `vehicle_imu` 实例读取数据；两者可以来自不同实例，不要求设备 ID 成对匹配。
从机 Airlink 不再直接发送 `sensor_accel` / `sensor_gyro`：使用
`delta_velocity / delta_velocity_dt` 得到平均比力，使用
`delta_angle / delta_angle_dt` 得到平均角速度，两个积分周期分别换算为秒。
这些数据已由 PX4 应用传感器标定、配置的温度补偿和板到 FRD 机体系旋转，
处于本地 EKF 输入层级，不扣除各机 EKF 的在线偏置。
从机传输保留原有定点量化：加速度分辨率 1/128 m/s²，角速度分辨率 1/800 rad/s。
选中 IMU 尚未发布或正在切换时不退回默认实例，也不发送旧传感器作为有效输入。

收到 Airlink 包不代表包内 IMU 有效。只有加速度、陀螺仪有效位和采样新鲜位均存在、
IMU 采样时间非零时，才使用从机输入。无效包不作为零向量参与 0.5 加权；
此时继续使用主机权重 1，收到有效从机输入后恢复各 0.5 并按建链流程初始化。
`CE_LINK` 表示 CEKF 可使用的从机输入状态，物理链路可另看 `airlink_status`。

两机 IMU 分别旋转到 C 系后：

| Airlink | 主机权重 | 从机权重 | CEKF 行为 |
| --- | --- | --- | --- |
| 连接 | 0.5 | 0.5 | 虚拟 IMU 更新 |
| 断开 | 1 | 0 | 主机 IMU 持续更新，不重置姿态 |
| 重连 | 0.5 | 0.5 | 立即以当前主机姿态重新初始化，恢复虚拟 IMU |

积分使用主机 IMU 采样时钟；主机新样本到达时使用最新从机数据。
从机新帧或建链时也尝试更新，同一个主机采样时刻只积分一次。
不再进行跨机时钟拟合和历史姿态插值。主机 IMU 保留正 dt、有限数值与重复采样检查，避免除零或重复积分。
可选杆臂补偿使用同样权重，角加速度由组合角速度差分低通获得。

## yaw 参考

仅发布 `composite_yaw_reference` 供 Airlink 传输。
两机各自如何使用 yaw 参考不在本模块处理范围内。
删除 `CEKF_YAW_INJ`、yaw-only `vehicle_odometry` 注入以及从机 yaw 接收处理。
删除不再使用的 `CEKF_LAT_US` 和 `CEKF_SRC_TOUT`。

## 命令

```sh
param set CEKF_EN 1
comp_attitude start
comp_attitude status
comp_attitude reset
listener composite_attitude_estimate
```

`reset` 仅重置独立 CEKF，不限制解锁状态。
删除旧 `bench`、`selftest` 命令；算法检查在主机测试脚本中运行。
