# H7Ultra 厂家 BSP 和 PX4 固件基线核验

厂家提供的项目：https://gitee.com/endlessflight/fc-h7-ultra  
用户上传压缩包：`fc-h7-ultra-master.zip`。

## 已实际校验的来源

- 外层 ZIP SHA-256：`9175a8738e712f23a860ada18cded7da120eaaf84ca3f5ca548aa5b3ee8ce7d0`。
- BSP 文件：`PX4/PX4_1.16.0/h743ultra.zip`；SHA-256：`b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27`。
- BSP 共 **24 个文件**：`default.px4board`、`firmware.prototype`、NuttX、board_config、DMA、I2C/SPI/PWM、传感器启动脚本、厂家引导加载程序等。
- 厂家预编译固件 `endlessflight_h743ultra_default.px4` 的 `git_hash`：`6ea3539157ca358c70a515878b77077af7d4611d`；与当前飞控 NSH `ver all` 一致。
- **该 Git SHA 恰好对应 PX4 官方 `v1.16.0` 标签所指向的提交。** 因此使用 [PX4/PX4-Autopilot v1.16.0](https://github.com/PX4/PX4-Autopilot/tree/v1.16.0) 作为本移植的完整源码基线，避免直接替换为 Ryan 的 1.16.2 变体。
- 厂家固件 `board_id=1179`，`description=Firmware for the EndlessFlight_H7ultra board`。**Board ID 不是唯一安全识别依据**，某些其他板（含 Ryan 原先的 MicoAir 配置）可复用 1179。
- 厂家固件 `image_size=1817128`，`image_maxsize=1835008`，余量只有 **17,880 字节**。新增模块之后必须重新完成链接尺寸检查。

## 厂家硬件事实

| 设备 | 硬件接口 |
| --- | --- |
| MCU | STM32H743VIT6 |
| IMU | 两颗 IIM42652 |
| 气压计 | SPA06-003 |
| 磁力计 | QMC5883P |
| TELEM1 / UART1 | `/dev/ttyS0`，RX1=Pin3，TX1=Pin4，GND=Pin2，5V=Pin1 |
| USB / QGC | `/dev/ttyACM0` |

两块板分别通过 USB 供电时，Pin1 5V 不连接，TX/RX 交叉且两端 GND 共地。

## GS-Link 源码与版本边界

移植源码来自 Ryan-5853/PX4-TeleaiTrans 固定提交
`3fef8242687cf733b590ff21c5b36779943299d1`。
所需文件已单独备份在本目录 `gs_link/` 和 `msg/` 下（见 [SOURCE_MANIFEST.md](SOURCE_MANIFEST.md)）。

Ryan 的原版模块已针对不同 PX4 版本实现，需要在 PX4 **官方 v1.16.0** 上交叉编译、检查 API；目前尚未完成该验证。通信模块订阅 `gs_link_status`，原系统由 `nn_control` 发布安全门控状态；第一阶段没有这个发布者，**不具备地面站控制授权**。

## 通信台架专用的保守处理

PX4 官方 v1.16.0 的树包含 IIM42652、SPA06、SCH16T 相关驱动，但是未提供厂家 BSP 引用的 `QMC5883P` 驱动，也未发现厂家特定 `COMMON_INS` Kconfig。

自动集成脚本只修改**工作区中的 BSP 副本**：

- 注释掉 `CONFIG_DRIVERS_MAGNETOMETER_QMC5883P=y` 和 `CONFIG_COMMON_INS=y`。
- 注释掉 `qmc5883p -I -a 0x2C -R 6 start`。
- 将 `param set-default SYS_HAS_MAG 1` 改为 `SYS_HAS_MAG 0`。
- 保留其他原厂引脚定义、NuttX、驱动映射，另建 `gslink.px4board` 测试变体。

厂家原 ZIP 不修改；不能用 QMC5883L 直接代替 QMC5883P。当前通信变体**仅限无桨台架**，不能飞行。

## 状态和后续验收

- [x] 原厂 ZIP、BSP、固件元数据 SHA 核对
- [x] 官方 PX4 v1.16.0 SHA 核对
- [x] 从 Ryan 源树提取 GS-Link 及 uORB 消息
- [x] 官方 PX4 v1.16.0 专用离线集成脚本和安全防覆盖逻辑
- [x] 模拟 PX4 checkout 下的 dry-run / apply / 幂等 / 冲突保护测试
- [ ] 真正 PX4 v1.16.0 全树 CMake 配置和 ARM 交叉编译
- [ ] Firmware 目标 Board ID、描述及 Flash 容量复核
- [ ] 无桨台架 USB QGC、UART1 和 ESP32-C5 / Dock_ground 双向通信测试
- [ ] QMC5883P 驱动及 nn_control / RC 门控移植、HIL/failsafe 验证

唯一推荐入口是 [integrate_px4_v1160.py](integrate_px4_v1160.py)。详细启动步骤见 [README.md](README.md)。**目前没有生成或烧录新固件。**
