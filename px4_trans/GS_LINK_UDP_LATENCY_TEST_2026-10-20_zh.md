# GS-Link UDP 延迟与带宽测试记录

日期：2026-10-20\
主机：Ubuntu，`enp5s0`，`192.168.31.220/24`，24 核\
ESP：`192.168.31.100`，DroneBridge for ESP32 v2.2.0 RC1（ESP32-C5）\
测试方向：主机 ↔ ESP；UDP 客户端临时注册在主机 `39052` 端口\
飞行控制：未发送 arm、takeoff、land 或任何执行器命令

## 1. ESP 当前配置

从 `http://192.168.31.100/api/settings` 和 `/api/system/info` 读取：

- Wi-Fi：5 GHz，channel 44，RSSI 约 -33 至 -35 dBm，power save 关闭；
- UART：`4000000` baud，protocol `5`（Transparent）；
- 最大包长：512 字节；
- serial timeout：1 ms；
- RTS/CTS：GPIO 0/0，未启用硬件流控；
- UDP client port 默认 39050；测试时临时改为主机 39052，测试后已清除；
- `serial_dec_mav_msgs=0`：Transparent 模式不解码 MAVLink，符合当前配置。

## 2. 网络基线

### ICMP RTT（200 包，20 ms 间隔）

| 指标 | 结果 |
| --- | ---: |
| min | 1.24 ms |
| p50 | 1.40 ms |
| p95 | 3.33 ms |
| p99 | 3.79 ms |
| max | 4.02 ms |
| mean | 1.88 ms |
| stdev | 0.77 ms |
| 丢包 | 0% |

同一地址前一轮 100 包测试的均值为 3.305 ms（min 2.705，max 5.142），说明 Wi-Fi
链路存在明显的时段性抖动，测试数据必须带环境记录。

### UDP 下行采样（ESP → 主机）

临时注册主机 UDP 客户端后采样 10.024 秒：

| 指标 | 结果 |
| --- | ---: |
| 包数 | 5919 |
| 字节数 | 18,588 |
| 包率 | 590.46 pps |
| 字节率 | 1,854 B/s |
| 包长 min/avg/max | 1 / 3.14 / 16 B |
| 包间隔 p50/p95/p99/max | 0.251 / 0.552 / 60.417 / 125.005 ms |
| 来源 | `192.168.31.100:14550` |

**这批 UDP 载荷全部是全零片段，不是有效 MAVLink 或 GS-Link 数据**。因此这组
包率/字节率只能说明 ESP→主机的 UDP 通道在工作，不能当作有效遥测带宽；60 ms
的 p99 包间隔主要反映 UART 零字节分片/突发，不是协议端到端延迟。

## 3. UART / PX4 侧检查

为判断端到端链路是否可用，做了以下只读/无飞行动作测试：

- TCP `5760`（DroneBridge MAVLink proxy）连接成功，但收到的前 257 字节全为 `00`；
- UDP 发送 20 个标准 MAVLink v2 `PING`，未收到任何 MAVLink 响应；
- ESP 的 `read_bytes` 持续增加，但 `serial_dec_mav_msgs=0`；
- 未发送控制类 MAVLink 消息。

结论：ESP 的 Wi-Fi/UDP 网络路径正常，但 ESP↔飞控 UART 没有可解析的有效数据。
最常见原因是：

1. ESP 与飞控串口波特率不一致；
2. ESP TX/RX 与飞控 RX/TX 未交叉，或未共地；
3. 飞控 TELEM1 没有运行 MAVLink，或飞控未上电；
4. 使用的是错误串口/引脚（ESP GPIO 11/12 是否对应实际接线）；
5. DroneBridge 的透明模式与飞控侧协议不匹配（当前飞控过渡固件仍是 MAVLink）。

## 4. 低延迟建议

- 保持 UDP；不要在控制回路上使用 TCP；
- ESP 已经使用 5 GHz channel 44、RSSI -33 dBm、power save 关闭，网络侧条件良好；
- 保持 `trans_pack_size=512` 和 `serial_timeout=1 ms`，避免大缓冲；
- 若 ESP32-C5 的 Wi-Fi 6/OFDMA 稳定，可对比测试 `wifi_ax=1`、`wifi_ofdma=1`，
  但必须用 p99 和丢包验证，不能只看平均值；
- 主机 UDP socket 建议设置 `SO_RCVBUF`/`SO_SNDBUF`、`SO_PRIORITY`，必要时使用
  `SO_BUSY_POLL` 和 CPU 绑核；
- GS-Link 的 50 Hz、约 3.7 kB/s 方案远低于 4 Mbaud 带宽，带宽不是主要瓶颈；
- 端到端延迟应在 UART 有效后重新测量，不能拿当前零字节流的分片间隔代替。

## 5. 复测前置条件

1. 飞控 TELEM1 与 ESP 都为 `4000000 8N1`；
2. ESP `proto=5`（Transparent）时，飞控侧应输出有效的 MAVLink 字节流；
3. 确认 ESP GPIO 11/12 与实际 TELEM1 TX/RX 交叉并共地；
4. 在飞控 NSH 检查 `MAV_0_CONFIG=101`、`SER_TEL1_BAUD=4000000` 和
   `mavlink status`；
5. 复测时先确认 ESP 收到 MAVLink heartbeat 或非零字节，再测 UDP RTT/吞吐。

测试结束后已清除临时 UDP 客户端；`udp_clients=[]`、`udp_connected=0`。

## 6. 烧录后复测（GS-Link 固件）

在刷入包含 `gs_link` 的 `micoair_h743-v2_nncontrol` 固件后复测：

- ESP 仍在线：ICMP 5/5，约 3.5–5.0 ms；
- DroneBridge 配置仍为 `4000000`、Transparent、512 B、1 ms timeout；
- ESP `/api/system/stats` 显示 `read_bytes=0`、`serial_dec_mav_msgs=0`；
- 临时注册主机 UDP 客户端后，4 秒内收到 `0` 个 UDP datagram；
- 发送 10 个合法 GS-Link `PING` 和一帧 GCS 心跳，未收到任何 ACK 或遥测；
- 测试后已清除临时客户端，`udp_clients=[]`。

这说明问题已经从“零字节噪声”变为“完全没有 UART 接收数据”。优先检查飞控是否
正在运行新固件、是否供电、ESP RX 是否接到飞控 TELEM1 TX、是否共地，以及
`gs_link status` 是否显示串口打开和 `tx_packets` 增长。若 `gs_link` 未自动启动，
可在 NSH 手动执行：

```text
param show MAV_0_CONFIG
param show NN_GS_EN
param show SER_TEL1_BAUD
gs_link status
listener gs_link_control
```

预期 `MAV_0_CONFIG=0`、`NN_GS_EN=0`、`SER_TEL1_BAUD=4000000`，`gs_link status`
应显示串口已打开且 TX 包计数增长。

烧录后的 NSH 实际输出确认了这一点：

```text
gs_link status
port: /dev/ttyS0, baud: 4000000, open: 1
session: 9e33ac3d, rx/tx packets: 0/10583
crc/length/version errors: 0/0/0
link: heartbeat stale, RC stale
```

因此飞控已经在 `/dev/ttyS0` 上持续发送 GS-Link 帧（约 10.5k 包），而 ESP 侧
`read_bytes=0`。问题不在 `gs_link` 启动或波特率参数，而在 ESP RX 引脚没有收到
飞控 TX 信号：重点检查 TELEM1 TX -> ESP GPIO12(RX)、TELEM1 RX <- ESP
GPIO11(TX) 两线是否交叉、是否共地，以及 ESP GPIO11/12 是否与实际接线一致。

`last error=9` 是 `sendPacket` 对非阻塞串口部分写入的旧错误；最新固件已加入
部分写入重试，但接线问题修复后仍需重新烧录该版本再复测。
