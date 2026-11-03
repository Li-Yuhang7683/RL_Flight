# AIRLINK v1 固件实现

日期：2026-10-30

本文对应 `AIRLINK_PROTOCOL_V1_zh.md` 的固件侧实现。实现位于
`px4_trans/px4/src/modules/airlink/`，默认角色为 disabled，不会因刷入新固件
而自动建立主从控制。

## 1. 已实现范围

- UART `6,000,000 8N1`、全双工、无硬件流控；
- 16 B 固定帧头、显式小端序列化、CRC-32C/ISCSI、EOF 和流式重同步；
- `SENSOR_STATE` 固定 42 B 打包，包含 IMU、姿态四元数、角速度估计、双 ESC
  RPM 和单电池电压；
- `ACTUATOR_COMMAND` 6 B、48 bit LSB-first 五路执行器命令；
- `HEARTBEAT`、三段式 `LINK_HANDSHAKE`、`CONTROL_ENABLE/RELEASE`、
  `PAIR_ARM/DISARM`、`EMERGENCY_WITHDRAW` 和 ACK；
- 可靠事件原序号重发、64 项/60 s 去重缓存、结果缓存和会话隔离；
- 500 Hz 心跳、500 Hz 传感器/控制流、10 ms 数据新鲜度、100 ms 预控制超时；
- `PAIR_ARM` 100 ms ACK 窗口、100 ms 首帧提交窗口、地面断链本地 disarm；
- `PAIR_REATTACH_LATCHED` 在当前启动周期内粘滞；
- 空中 `PAIR_LOST` 后进入 `SINGLE_LANDING`：先尝试恢复 GS-LINK，否则以
  collective `0.25` 保持水平姿态下降；冲击、翻转或超时触发本地 disarm；
- `LINK_TEST_CONTROL`/`LINK_TEST_DATA` 回环测试、丢包/重复/乱序、
  RTT 分位数、吞吐和最大接收间隔统计；
- QGC `AIRLINK_ROLE`、`AIRLINK_PAIR_EN` 参数和 NSH 命令；
- PX4 Commander arming check 接入，配对模式下不能绕过 `PAIR_ARM` 直接 arm；
- `airlink_status`、`airlink_actuator_command`、`airlink_pair_command` 和
  `airlink_test_result` uORB 接口。

## 2. 固件组成

| 文件 | 作用 |
| --- | --- |
| `AirlinkProtocol.hpp` | 固定 ABI、CRC、帧解析、字段量化、全部消息编解码 |
| `AirlinkSession.hpp/.cpp` | 发现、握手、可靠事件、arm 生命周期、新鲜度和链路测试状态机 |
| `Airlink.hpp/.cpp` | UART、uORB、参数、Commander arm/disarm、控制和诊断发布 |
| `airlink_params.c` | 角色、配对模式和 SINGLE_LANDING 安全阈值 |
| `AirlinkStatus.msg` | 会话、链路状态、错误计数和本地安全门控 |
| `AirlinkActuatorCommand.msg` | 从机收到的五路控制器域命令 |
| `AirlinkPairCommand.msg` | C_PAIR 控制器向固件提供的本机/从机五路命令 |
| `AirlinkTestResult.msg` | 链路测试完整机器可读结果 |

## 3. 板级端口与参数

`micoair_h743-v2_nncontrol` 启动脚本将 AIRLINK 配置为：

```text
device = /dev/ttyS1
baud   = 6000000
format = 8N1, no flow control
```

`/dev/ttyS1` 是该板卡映射中的 UART2/GPS2，`/dev/ttyS0` 已由 GS-LINK 使用。
两端 TX/RX 必须交叉并共地。

参数默认值：

```text
AIRLINK_ROLE=0       # 0 disabled, 1 master, 2 slave
AIRLINK_PAIR_EN=0    # 0 保留单机控制，1 选择双机控制
AIRLINK_IMPACT=35    # m/s²，初始经验值
AIRLINK_IMP_MS=20    # 连续冲击确认时间
AIRLINK_FLIP_DEG=70  # 相对竖直方向，初始经验值
AIRLINK_FLIP_MS=150  # 连续翻转确认时间
AIRLINK_LAND_MS=15000 # SINGLE_LANDING 强制 disarm 时限
```

`AIRLINK_ROLE` 和 `AIRLINK_PAIR_EN` 只能在 disarmed、无 `PAIR_ACTIVE` 和无
链路测试时修改。armed/active 时 QGC 写入会被固件拒绝并恢复原值。

五个降落阈值参与握手 `config_crc32c`。两机必须使用相同数值；修改后旧会话必须
重新握手。`nn_control` 在 disarmed 阶段锁存这些阈值，armed 后修改不会影响当前
flight epoch。默认值仅用于首轮 HIL/台架，不是飞行标定结果。

## 4. 操作命令

NSH/QGC MAVLink Console：

```sh
airlink help
airlink status
airlink role master
airlink role slave
airlink role off
airlink pair on
airlink pair off
```

主从配置顺序、状态判据、主题监听、链路测试和故障排查的完整 PX4 端操作说明见
[`../px4_trans/README.md`](../px4_trans/README.md) 的“AIRLINK v1 PX4/NSH 调试指令”一节。

配对 arm 事务：

```sh
airlink arm
airlink disarm
```

QGC `MAV_CMD_COMPONENT_ARM_DISARM` 和 RC/摇杆 `action_request` 的 arm 请求也会
触发同一事务。配对模式下，Airlink 会先完成 fresh `CONTROL_ENABLE`，随后发送
`PAIR_ARM`；从机真正进入 PX4 armed 后才回成功 ACK，主机收到 ACK 后才执行本机
arm。

控制释放和紧急撤销：

```sh
airlink release
airlink emergency
```

## 5. 控制权接口

`airlink` 不直接绕过 `nn_control` 写执行器。命令所有权由以下规则执行：

- master 在 `PAIR_ARMED_GROUND/PAIR_ACTIVE` 只使用新的
  `airlink_pair_command.master_*`；
- slave 在 `PAIR_ACTIVE` 只使用 10 ms 内新鲜的
  `airlink_actuator_command`；
- 其他状态恢复 `nn_control` 的 C_SINGLE 输出；
- 两种模式不会在同一控制周期混合写入执行器；
- 本地 RC、Commander、failsafe、`NN_LQI_OUTPUT_EN` 和 airframe 配置门仍然有效。

`AirlinkPairCommand.msg` 是 C_PAIR 控制器与 AIRLINK 固件之间的稳定接口。
当前仓库尚无发布该主题的 C_PAIR 控制器实现，因此没有 C_PAIR 发布者时：

- `CONTROL_ENABLE` 不会由自动流程发起；
- `PAIR_ARM` 会被拒绝；
- 飞机保持单机控制或 disarm。

这是有意的 fail-closed 行为，不是通信层的失败。

## 6. 链路测试

默认参数：

```sh
airlink test --duration-ms 5000 --rate-hz 500 --payload-bytes 256
airlink aborttest
```

测试仅允许 master、`AIRLINK_READY`、两端 disarmed 时执行。退出后结果同时发布到
`airlink_test_result`，并在控制台输出完整 `AIRLINK_TEST_JSON=...`。其中
`buffer_overflows` 必须为零；非零表示应用层单帧接收缓冲被错误写入，而不是可以
通过重试消除的普通网络丢包。MAVLink `STATUSTEXT` 因 PX4 50 B 限制输出关键摘要，
完整字段以 uORB 主题和控制台日志为准。

AIRLINK 工作项按 1 kHz 读取和解析接收数据，但 `SENSOR_STATE`、
`ACTUATOR_COMMAND`、心跳、会话 tick 和状态发布继续按 2 ms 节流，数据流协议
速率仍为 500 Hz。

## 7. 验证

主机协议和双端状态机测试：

```bash
bash px4_trans/tests/run_airlink_protocol_check.sh
```

覆盖：

- CRC-32C 自检和协议文档全部 64 B 金样例；
- 任意拆包、粘包、假 SOF、payload 内 SOF、坏 CRC/EOF/version/role；
- 传感器量化边界、饱和位、四元数、可选 ESC 和电压；
- 五路执行器最小/中点/最大位流及非法码；
- 三阶段握手、配置不一致、角色冲突、可靠重试；
- `PAIR_ARM`、首次控制接管、重复 arm、10 ms 流超时和重连锁存；
- `SINGLE_LANDING` 冲击、翻转确认时间和总超时边界；
- 未知 COMMAND 的 `UNSUPPORTED` ACK 和链路测试回环。

固件构建：

```bash
env \
  CCACHE_DISABLE=1 \
  MAKEFLAGS= \
  PATH="$PWD/px4_trans/px4/.venv/bin:$PATH" \
  PYTHON_EXECUTABLE="$PWD/px4_trans/px4/.venv/bin/python" \
  make -C px4_trans/px4 micoair_h743-v2_nncontrol
```

输出：

```text
px4_trans/px4/build/micoair_h743-v2_nncontrol/micoair_h743-v2_nncontrol.px4
```

## 8. 尚未构成飞行许可的边界

- 尚未在真实 MicoAir H743 上验证 6 Mbaud 误码、DMA 缓存、连接器半插入和
  运行中拔线；
- 当前仓库没有 C_PAIR 控制算法发布者；
- 冲击、翻转和 landing 超时已经提供可配置经验默认值；这些值尚未按当前机架
  质量、减振和下降速度完成台架/HIL 标定；
- 编译与主机测试通过只证明协议、状态机和固件集成可构建，不等同于 HIL、台架、
  实机或飞行许可。
