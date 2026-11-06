# Airframe-3 新组装共轴双电机＋三舵机：参数辨识、LQI 与 PX4 安全交付

> 状态：仅工程草案／未校准。**没有真机参数、没有 airframe-3 已验证增益、没有编译出来的 airframe-3 固件**。
> 本分支禁止把机架1的物理数据当作机架3实测数据使用；不允许作为实飞放行依据。

## 1. 设计目标与合同

- 机体稳定身份：`airframe-3`；机架编号 `3`，不由飞控 UUID 代替。
- 机体：两个共轴反转电机 + 三格栅舵机；对齐 `NN_LQI_MODEL=2` 的 **8 状态姿态 LQI + Z 轴惯性加速度 PI** 合同。
- 姿态状态向量：roll、pitch 姿态误差，三个角速度误差，三个积分状态。
- 姿态 LQI 输入：roll / pitch / yaw 虚拟力矩；两个电机负责总距和 yaw 差动，三舵机承担 roll/pitch。
- 设定世界坐标 NED、机体 FRD、四元数 wxyz；世界 NED 垂直加速度与机体加速度计比力不同。
- 这是控制器设计框架，不等于“一套 LQI 能同时估计所有物理参数”。离线 NN 负责控制等效响应辨识，实测与动力学辨识负责质量、惯量、总推力、电机响应等；LQI 消费这些模型参数。

## 2. 真实测量与校核

必须建立 `Identification/configs/airframe3_parameter_intake.yaml`，填写真实证据来源和不确定度。
1. 精确称总质量、定位质心；通过几何/摆法/台架测量或保守辨识得到三轴惯量及合理物理边界。
2. 分别标定上下电机的物理输出指令 → 机械转速曲线，确认 DShot、ESC 供电、电机磁极数量及 RPM 换算；单独记录 **两份 CSV**。不得直接使用机架1上桨曲线作为双桨替代。
3. 测试上/下桨推力与差动反扭矩（理想上通过固定台架与多电流/电压点），分离仅转速标定与推力曲线；识别各电机惯性时间常数。
4. 记录三舵机打角、限速、死区、回差、舵角方向；检查气动格栅几何、舵效（至少小信号极性与约束）。
5. 固定传感器坐标与时序，验证 PX4 uORB 采样、SD 记录质量、状态估计器、解锁与失效保护。
6. 新机先完成不带桨输出映射核对、台架标定及已有安全控制器的约束验证；禁止拿未校准 LQI 首飞来“产生数据”。

`airframe-3` 的真实物理模型应独立保存到
`SimEnv/configs/sim2real_micro_coaxial_airframe3.yaml`（完整 schema 与 SimEnv 对齐），
且标记测量与模型不确定性。该文件故意未生成，以免占位数字被误认成真实模型。

## 3. 数据集与离线辨识

仅在既有控制器已获验证且飞行许可/安全措施到位后，收集真实闭环日志。
建立独立的 `log/airframe-3/first_identification/` 目录，规范数据集放
`Identification/datasets/airframe-3/first_identification_v1/`。
旧版真实日志辨识 `offline_log_inference.py` 支持主 checkpoint + 单个辅助 checkpoint；
当前三网络 TCN-roll、BiGRU-pitch、MLP-yaw 要扩展真实日志推理的第二辅助 checkpoint
才可按三轴混合。必须完成质量门控、OOD 检查与执行器饱和检查，且记录数据、代码、checkpoint SHA256。

## 4. 控制综合与部署门控

- 以校准参数生成各机体的 A/B（或姿态力矩惯量模型）、控制分配和悬停工作点；确认可控性及极性。
- 解离散 DARE：`K = dlqr(A_d,B_d,Q,R)`；评估本机真实模型不确定性下的极点、饱和、收敛和故障回退。
- `Controller/configs/lqi_coaxial_collective_servo_airframe3.yaml` 的 Q/R 尺度目前沿用**设计起点**，并非已调好。
- 在固件生成配置 `px4_trans/configs/lqi_coaxial_collective_airframe3.yaml` 中，
  `calibration_ready: false` 保持拒绝生成；缺数据严禁改成 true。
- 源码生成器现在支持新机标识和两份独立转速 CSV，但新机激活要求显式 `--activate`。
  即使生成器成功，也**不意味着** `flight_authorized=true`；清单始终记录 false。
- 经过参数、仿真、HIL、审查后，在独立分支上执行生成（此处只是预期命令，非已经执行）：

```bash
PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
python px4_trans/tools/generate_coaxial_collective_backend.py \
  --deployment-config px4_trans/configs/lqi_coaxial_collective_airframe3.yaml \
  --output-dir px4_trans/generated/airframe-3 \
  --no-activate
```

- 确认正确、完成 host golden-vector 测试，才可使用显式 `--activate` 将候选载荷放入
`px4_trans/px4/src/modules/nn_control/`，并用目标板工具链构建 `micoair_h743-v2_nncontrol`。
- 固件必须满足 `NN_AIRFRAME_ID=3` 与内嵌编号一致、`NN_LQI_MODEL=2`，
  `NN_LQI_OUTPUT_EN=0` **默认禁止执行器输出**；仅在完整放行流程里考虑解除。
- 禁止把已编译固件/控制器模型的有限值或极点检查视为真实飞行稳定证明。

## 5. 当前遗留开发任务

1. 将旧版 `offline_log_inference.py` 扩展为三个 checkpoint 的真实日志逐轴融合；
2. 补充生成器的空气动力、电机力矩和机体参数来源的更严格数量/范围/不确定度校核；
3. 按 airframe-3 独立实测建立完整 SimEnv 配置与真实日志辨识报告；
4. 运行独立参数扰动闭环、接口 HIL、主机 C++ golden-vectors 和目标板构建；
5. 逐项审查 MCU 输出路径、传感器新鲜度、失效保护与人工回退；
6. **在没有实测数据之前，任何输出都只能是研究/候选用途，不构成飞行授权。**
