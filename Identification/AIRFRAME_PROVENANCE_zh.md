# 多机架辨识与 LQI 绑定契约

## 1. 当前分配

| 机架 | 稳定 ID | 当前用途 |
| --- | --- | --- |
| 机架1 | `airframe-1` | 2026-10-26 闭环日志、`pq03` 辨识结果、checksum `0x1716d847` |
| 机架2 | `airframe-2` | 正在重新采集；`log/flight2` 仅作前期故障诊断 |

机架注册表是 [airframes.yaml](configs/airframes.yaml)。`airframe_id` 是程序使用的稳定键，
`airframe_number` 是写入飞控参数 `NN_AIRFRAME_ID` 的正整数。`display_name` 只用于人阅读。

飞控硬件 UUID 不能代替机架 ID。同一个飞控可以换装到不同机体，同一机架也可能更换飞控，
因此日志、辨识结果和模型必须依靠显式机架身份，而不能根据 ULog 的 `sys_uuid` 推断。

## 2. 贯穿链路

机架身份沿以下链路传播，并在每个边界校验：

```text
airframes.yaml
    airframe_id / airframe_number
            |
            v
datasets/<dataset>/airframe.json + bundle.npz
            |
            v
offline_identification*.json
            |
            v
LqiIdentifiedModel.hpp + generated/<airframe_id>/lqi_embedded_model.json
            |
            v
NN_AIRFRAME_ID == embedded kAirframeNumber
```

任何一层发现 `airframe_id` 不一致都应停止生成，而不是仅打印警告。

## 3. 数据集接口

构建真实日志数据集时必须指定机架：

```bash
env PYTHONNOUSERSITE=1 \
  PYTHONPATH=Identification/src:Controller/src:SimEnv/src:px4_trans/px4/.venv/lib/python3.10/site-packages \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  Identification/scripts/build_real_log_dataset.py \
  --logs log/26.10.26 \
  --output Identification/datasets/real_logs_26_10_26_v4 \
  --airframe-id airframe-1 \
  --recursive
```

输出目录包含：

- `airframe.json`：数据集所属机架及其绑定信息；
- `bundle.npz`：数据数组，同时嵌入 `airframe_id` 和 `airframe_json`；
- `build_report.json`、`windows.json`：顶层 `airframe` 字段。

新机架的数据不要追加到已有目录。机架2应使用独立目录，例如：

```text
Identification/datasets/airframe-2/2026-xx-xx_v1
```

原始 ULog 同样应放入新的专用目录（例如 `log/airframe-2/2026-xx-xx/`），
并为该目录创建 `airframe.json`，其中
`use_for_identification=true`。不要直接把 `log/flight2` 的
`use_for_identification=false` 改成 `true`，否则会把前期失稳日志与新数据混成一套。

## 4. 辨识结果接口

离线辨识报告必须包含：

```json
{
  "schema_version": 2,
  "airframe": {
    "airframe_id": "airframe-1",
    "airframe_number": 1,
    "display_name": "机架1"
  }
}
```

CLI 会从数据包读取机架身份；若显式传入 `--airframe-id`，两者必须一致：

```bash
env PYTHONNOUSERSITE=1 \
  PYTHONPATH=Identification/src:Controller/src:SimEnv/src:px4_trans/px4/.venv/lib/python3.10/site-packages \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  -m flight_identification.offline_log_inference \
  --airframe-id airframe-1 \
  --logs Identification/datasets/real_logs_26_10_26_v4/bundle.npz \
  --checkpoint <primary-checkpoint> \
  --experiment-config <experiment.yaml> \
  --output Identification/datasets/airframe-1/<result>.json
```

报告同时记录日志包、检查点和质量标定文件的 SHA-256。机架身份不等于飞行放行：
`deployment_mode` 和 `gain_accepted_for_flight` 仍按原规则独立判断。

## 5. LQI 固件模型接口

机架1的规范部署配置是 [lqi_airframe_1.yaml](../px4_trans/configs/lqi_airframe_1.yaml)：

```yaml
airframe_id: airframe-1
airframe_registry: Identification/configs/airframes.yaml
identified_model:
  report: Identification/datasets/real_logs_26_10_26_v4/offline_identification_v7_hybrid_pq03.json
  binding_checkpoint: Identification/runs/sim2real_offline_logs_step_response_tcn_500hz_v7/identifier.pt
  expected_checksum: "0x1716d847"
```

生成固件模型：

```bash
env PYTHONNOUSERSITE=1 \
  PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  px4_trans/tools/generate_lqi_backend.py
```

生成器会拒绝以下情况：

- 辨识报告没有 `airframe`；
- 报告机架与部署配置或注册表不一致；
- 生成 checksum 与 `expected_checksum` 不一致；
- 检查点或报告不存在。

生成结果包括：

- `LqiIdentifiedModel.hpp` 中的 `kAirframeId`、`kAirframeNumber`、
  `kIdentificationArtifactSha256`；
- `Identification/datasets/airframe-1/<result>.json` 的 SHA-256；
- `px4_trans/generated/airframe-1/LqiIdentifiedModel.hpp` 归档副本；
- `px4_trans/generated/airframe-1/lqi_embedded_model.json` 完整来源清单。

固件构建完成后，将二进制 SHA-256 写回同一清单：

```bash
/home/ryan/miniconda3/envs/rl-flight/bin/python \
  px4_trans/tools/stamp_lqi_firmware.py \
  --airframe-id airframe-1 \
  --firmware px4_trans/px4/build/micoair_h743-v2_nncontrol/micoair_h743-v2_nncontrol.px4
```

这样可以从一个清单同时追到机架、日志数据包、辨识报告、检查点、模型头文件、
模型 checksum 和最终固件 SHA-256。

不同机架应使用各自的部署 YAML。当前固件只编译一个活动
`LqiIdentifiedModel.hpp`，但每个机架生成的快照会保留在
`px4_trans/generated/<airframe_id>/`，不得用机架2的数据覆盖后丢失机架1溯源。

## 6. 运行时接口

辨识模型内嵌预期的 `airframe_number`。`nn_control` 启动时读取
`NN_AIRFRAME_ID`：

```text
NN_AIRFRAME_ID == model.airframe_number
```

不相等时，即使 `NN_LQI_OUTPUT_EN=1`，执行器输出仍保持关闭，原因显示为
`identified model airframe mismatch`。标称 `NN_LQI_MODEL=0` 是机架无关模型，
不执行该检查。

部署到机架2前，必须显式设置并保存：

```text
param set NN_AIRFRAME_ID 2
param save
reboot
```

启动 ULog 的 boot console 和 `nn_control status` 都会显示：

- 当前模型机架 ID；
- 模型预期编号；
- 参数配置编号；
- 是否不匹配；
- 辨识报告 SHA-256；
- 模型 weights checksum。

`NN_AIRFRAME_ID` 只防止错刷模型。它不会替代执行器标定、HIL、系留测试或飞行放行检查。

## 7. 查询注册表

```bash
PYTHONPATH=Identification/src \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  -m flight_identification.airframe list

PYTHONPATH=Identification/src \
  /home/ryan/miniconda3/envs/rl-flight/bin/python \
  -m flight_identification.airframe show airframe-1
```
