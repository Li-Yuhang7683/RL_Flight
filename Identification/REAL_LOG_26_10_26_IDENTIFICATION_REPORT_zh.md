# 2026-10-26 新机械/电池构型真机日志辨识报告

## 机架身份

- `airframe_id`：`airframe-1`
- 名称：机架1
- 注册表：`Identification/configs/airframes.yaml`
- 当前部署配置：`px4_trans/configs/lqi_airframe_1.yaml`
- 当前标识模型：`0x1716d847`

本文的数据、辨识结果和 `pq03` LQI 参数只属于机架1。`log/flight2` 的
机架2诊断日志没有混入本数据集。

## 结论

已按 500 Hz、2 s 整窗的 14 通道契约完成日志清洗，并使用 v7
TCN(roll) + BiGRU(pitch) + MLP(yaw) 混合辨识和 80% 增益混合生成新的
4x19 复合 LQI 权重。新权重已通过 Python/C++ 黄金向量一致性检查、v2 部署配置下的
闭环极性与回正检查，并完成 `micoair_h743-v2_nncontrol` 固件构建。

但本次日志的信息量和输入分布 **没有通过既有质量门控**，因此发布等级仍然是
`research_only / offline_analysis_only`，`gain_accepted_for_flight=false`。
生成的固件是待 HIL/shadow 验证的候选，不可仅凭本次辨识结果直接用于实机接管。

## 日志清洗

- 输入：`log/26.10.26`，12 个唯一 ULog，无重复文件。
- 12 份日志的 `sys_hw` 均为 `MICOAIR_H743_V2`，固件提交均为
  `b103229335dc568d8ed41ea6b9e9bc8d898990d8`，因此可作为同一构型批次处理。
- `log_0` 缺少 `vehicle_attitude`，不能进入辨识。
- `log_2` 没有有效的持续在控段。
- 其余部分日志存在 ESC/执行器覆盖空洞、短于 2 s 的在控段或大幅倾覆，未作为整窗
  数据使用。
- 清洗器新增对每个“连续有效且在控”运行段做 50 ms 步长扫描；仍保留 500 Hz、
  1000 样本整窗、全样本有效、倾角/角速率上限、起飞/扰动事件优先级及窗口非重叠
  约束。
- 最终选出 8 个 Tier-A/B 窗口，来源计数为 `log_3` 1 个、`log_6` 1 个、
  `log_7` 1 个、`log_11` 5 个。

规范化产物位于 `Identification/datasets/real_logs_26_10_26_v1/`。

## 辨识与质量门控

混合模型和 80% 增益混合的分析结果：

| 指标 | 结果 |
| --- | ---: |
| 窗口数 | 8 |
| 聚合 roll/pitch/yaw 角速率 RMS | 0.3106 / 0.2294 / 0.2909 rad/s |
| 门限 roll/pitch/yaw 角速率 RMS | 0.2816 / 0.2735 / 0.1976 rad/s |
| 指令运动量 RMS | 0.4749 |
| 指令运动量门限 | 0.0511 |
| 输入 `abs(z)` p95 | 4.0999 |
| 输入 `abs(z)` p95 门限 | 2.9956 |
| 输入 `abs(z)` max | 6.6308 |
| 输入 `abs(z)` max 门限 | 13.0441 |
| 标称模型极点半径 | 0.995572 |
| 辨识模型极点半径 | 0.995643 |

门控失败项是 pitch 角速率覆盖和输入分布 p95。roll/z 角速率覆盖及指令运动量满足
门限。继续生成权重是为了保留可复核的新构型候选，不代表质量门控通过。

## 部署载荷与验证

- 标称配置：`px4_trans/configs/lqi_nominal_deployment_v2.yaml`
- 标称模型 checksum：`0x29f36f40`
- 辨识 4x19 模型 checksum：`0xfe835dba`
- 辨识模型权重字节数：556
- 辨识黄金向量：5
- `run_lqi_core_check.sh`：标称和辨识核心均通过
- 闭环极性：roll/pitch 正负方向角加速度符号全部正确
- 输出：全部有限且在各执行器范围内
- 名义/辨识 roll 方向一致
- 4 s 回正末倾角：0.0394-0.3683 deg
- 正负 roll 舵机指令反对称误差：0.01513 deg

固件：

```text
px4_trans/px4/build/micoair_h743-v2_nncontrol/micoair_h743-v2_nncontrol.px4
```

SHA-256：

```text
ed45ed676f8c9c984a25574c3fd9e3db69abbc862b845345e82c1be397e4c009
```

## 备份

替换前的 19/13 状态头文件、黄金向量、旧固件和 v7 检查点保存在：

```text
Identification/backups/20261026_pre_mechanical_battery_update.tar.gz
```

归档 SHA-256：

```text
8caecfd6e601fd1b4d248fd1742bcc477e8f098dc246119d1679443f57a7eaca
```

本次新清洗/辨识数据另存为：

```text
Identification/backups/real_logs_26_10_26_v1_data.tar.gz
```

SHA-256：

```text
5bf4b20d022f264411e7edcee038f59082f84ce337b3e193f80970db43be8f49
```

## 后续放行条件

需要补充至少多批包含 roll、pitch、yaw 三轴闭环激励且执行器/ESC 覆盖完整的数据，
重新运行质量门控。候选固件还必须经过 HIL、限幅/饱和检查、未知初始条件仿真和实机
shadow 飞行，才能讨论飞行授权。

## 补充批次 `26.10.26/1`

后续加入 `log/26.10.26/1` 的 13 份 ULog，并对上下两层递归清洗。两层均有 1 份
缺少 `vehicle_attitude` 的 `log_0`，其余 25 份唯一日志参与解析。上下两层会重名，
因此数据源统一记录为相对路径，例如 `1/log_2_UnknownDate.ulg`。

组合数据先得到 `real_logs_26_10_26_v2`：32 个窗口，三轴角速率覆盖均超过门限，
但包含饱和占比最高达 1.0 的窗口，且 `abs(z)` p95 为 4.156。随后增加“2 s 窗口
执行器饱和占比不超过 50%”的清洗门限，得到 `real_logs_26_10_26_v3`：21 个窗口，
top-8 聚合结果为：

| 指标 | 结果 | 门限 |
| --- | ---: | ---: |
| roll/pitch/yaw 角速率 RMS | 0.5597 / 0.4931 / 0.2840 | 0.2816 / 0.2735 / 0.1976 |
| 指令运动量 RMS | 0.9114 | 0.0511 |
| 输入 `abs(z)` p95 | 3.9995 | 2.9956 |
| 输入 `abs(z)` max | 10.0257 | 13.0441 |
| 辨识模型极点半径 | 0.995591 | - |

新增数据显著改善了三轴激励覆盖和输入分布最大值，但 `abs(z)` p95 仍超限，质量门控
仍未通过。尝试生成的新组合模型 checksum 为 `0x04b0ad54`；其 ±roll 舵机反对称
误差为 `0.04495`，超过 `0.02` 门限。对 v3 系数取中位数的误差为 `0.03026`，
将 roll 系数完全替换为 v1 后仍为 `0.02564`，均未通过。

因此没有用组合日志模型替换已验证固件。当前 `LqiIdentifiedModel.hpp` 和固件仍恢复
为 v1 载荷，checksum `0xfe835dba`，固件 SHA-256 仍为
`ed45ed676f8c9c984a25574c3fd9e3db69abbc862b845345e82c1be397e4c009`。
极性脚本已修正为反对称失败会同步令 `polarity_ok=false` 并返回非零退出码。

组合尝试的完整窗口、v2/v3 辨识结果和未采用模型已归档：

```text
Identification/backups/20261026_combined_logs_v3_rejected_model.tar.gz
```

SHA-256：

```text
25191cd08474ce96376a7db8ad2be683a64577bdc41dc2a7240b865b4c9bb8a6
```
