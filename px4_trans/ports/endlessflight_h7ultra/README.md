# H7Ultra / PX4 1.16.0 / Ryan GS-Link v1 — 第一阶段

**状态：官方 PX4 1.16.0 + 寰翼 H7Ultra BSP + Ryan GS-Link 的本地自动集成脚本已提交。完整 PX4 交叉编译、刷机和实机台架验证尚未完成；不得用于飞行。**

## 1. 固定源码基线

| 项目 | 来源 |
| --- | --- |
| PX4 核心 | [官方 PX4 v1.16.0](https://github.com/PX4/PX4-Autopilot/tree/v1.16.0)，Git SHA `6ea3539157ca358c70a515878b77077af7d4611d` |
| 寰翼 BSP | 用户上传 `fc-h7-ultra-master.zip` 内 `PX4/PX4_1.16.0/h743ultra.zip`，共 24 个板级文件 |
| BSP SHA-256 | `b20f875ea87e09134e4cf9727273684ded0c45cff5c6543c022cb449bf28cc27` |
| GS-Link v1 | Ryan-5853/PX4-TeleaiTrans @ `3fef8242687cf733b590ff21c5b36779943299d1` |
| 最终板卡 | `boards/endlessflight/h743ultra`；UART1/TELEM1=`/dev/ttyS0` |
| QGC 调试 | USB `/dev/ttyACM0`，独立保留 |

**重要纠正：** 原厂预编译固件的 Git 哈希和官方 PX4 v1.16.0 完全一致。**本新固件应以官方 v1.16.0 为核心**，不是把 Ryan 为 MicoAir 修改的 1.16.2 整体移植到 H7Ultra。原来的 `px4_trans/px4` 子模块不修改。

## 2. Windows / WSL2 操作（请按顺序）

**先打开 Windows Terminal / PowerShell：**

```powershell
wsl -d Ubuntu-24.04
```

**以下命令全部在 Ubuntu 终端执行。** 先检查是否有未保存的本地改动，再拉取已提交的移植分支：

```bash
cd ~/projects/RL_Flight
git status --short
git fetch origin feature/endlessflight-h7ultra-gs-link
git switch feature/endlessflight-h7ultra-gs-link
git pull --ff-only origin feature/endlessflight-h7ultra-gs-link
```

如果新分支在本地不存在，改为 `git switch --track origin/feature/endlessflight-h7ultra-gs-link`。若 `git status` 有未提交修改，先不要切换。

创建**新的独立 PX4 核心目录**，不要拿原 Ryan 子模块当作 H7Ultra 基线：

```bash
cd ~/projects
git clone --branch v1.16.0 --recurse-submodules \
  https://github.com/PX4/PX4-Autopilot.git PX4-H7Ultra-GSLink
git -C ~/projects/PX4-H7Ultra-GSLink rev-parse HEAD
```

最后一行**必须**是 `6ea3539157ca358c70a515878b77077af7d4611d`。

将厂家 ZIP 复制到能在 WSL 中读到的位置。下面以 Windows 路径 `D:\BaiduNetdiskDownload\fc-h7-ultra-master.zip` 为例；如实际位置不同，请先自行调整：

```bash
cd ~/projects/RL_Flight
VENDOR_ZIP=/mnt/d/BaiduNetdiskDownload/fc-h7-ultra-master.zip
ls -lh "$VENDOR_ZIP"
```

**只读预演：**

```bash
python3 px4_trans/ports/endlessflight_h7ultra/integrate_px4_v1160.py \
  --vendor-zip "$VENDOR_ZIP" \
  --px4-dir ~/projects/PX4-H7Ultra-GSLink
```

**核对输出后，再正式写入另一套 PX4 源码目录：**

```bash
python3 px4_trans/ports/endlessflight_h7ultra/integrate_px4_v1160.py \
  --vendor-zip "$VENDOR_ZIP" \
  --px4-dir ~/projects/PX4-H7Ultra-GSLink --apply
```

该操作只修改电脑上的 **PX4 源码 checkout**，不连接或修改硬件。安装后检查：

```bash
git -C ~/projects/PX4-H7Ultra-GSLink status --short
grep GsLink ~/projects/PX4-H7Ultra-GSLink/msg/CMakeLists.txt
cat ~/projects/PX4-H7Ultra-GSLink/boards/endlessflight/h743ultra/gslink.px4board
```

准备好官方 PX4 1.16.0 编译依赖后，在 **Ubuntu 终端**尝试构建：

```bash
cd ~/projects/PX4-H7Ultra-GSLink
make endlessflight_h743ultra_gslink
```

此构建命令**尚未在完整官方 PX4 环境中运行验证**。如失败，保存从第一个 `error:` 开始的完整输出；不要烧录任何未验证的 PX4 文件。

## 3. 集成内容与已知硬件限制

唯一推荐脚本：[integrate_px4_v1160.py](integrate_px4_v1160.py)。

脚本精确检查官方 v1.16.0 Git SHA、厂家 BSP SHA-256、Board 描述及 UART1 映射，然后：

1. 从厂家 ZIP 提取完整 **24 个** H7Ultra BSP 文件到 `boards/endlessflight/h743ultra`。
2. 移入固定 Ryan 版本的 GS-Link `GsLink.cpp/GsLink.hpp/GsLinkProtocol.hpp/CMakeLists.txt/Kconfig`。
3. 添加 `GsLinkControl.msg`、`GsLinkStatus.msg`，并注册到 PX4 `msg/CMakeLists.txt`。
4. 新建 `gslink.px4board` 构建目标，只启用 GS-Link 通信；不自动占用 UART1。
5. **仅在工作区 BSP 副本**注释掉官方 PX4 不支持的 `QMC5883P` 和 `COMMON_INS` 配置、注释掉 `qmc5883p` 启动行、设置 `SYS_HAS_MAG=0` 默认值。原厂家 ZIP 不变。
6. 为有限 Flash 关闭部分非通信/非多旋翼模块；仍需实际编译确认能否装入 Flash。

原厂预编译固件已经使用 `1,817,128 / 1,835,008` 字节，剩余只有 **17,880 字节**，因此新增 GS-Link 后的链接结果必须重新测量。

**明确不能飞：** QMC5883P 驱动尚未移植；GS-Link 所需 `nn_control` 安全门控发布/消费路径也未移植；没有跑过固件编译、HIL、台架、断链测试。即使遥测成功也不代表地面站具有任何控制权限。

## 4. ESP32-C5 和电脑端

- H7Ultra UART1 TX Pin4 → ESP32-C5 GPIO12 RX；RX Pin3 ← ESP32-C5 GPIO11 TX；Pin2 → GND。两块板分别 USB 供电时，**Pin1 5V 不连接**。
- 首轮串口：PX4 和 ESP32-C5 **统一 921600 baud**；Ryan 原版 4 Mbaud 不是当前必要条件。
- ESP32-C5 端口 `UDP 14550`，电脑端 [Dock_ground](https://github.com/Ryan-5853/Dock_ground) `UDP 39050`。这是 **GS-Link v1**，不是直接交给 QGC 的 MAVLink。
- USB QGC `/dev/ttyACM0` 继续提供参数与 NSH。
- **当前飞控仍运行原厂 MAVLink，不要现在修改参数。** 只有在编译、验证并准备无桨通信台架之后，才能在新固件上禁用 TELEM1 MAVLink，并手动运行 `gs_link start -d /dev/ttyS0 -b 921600`，检查 `gs_link status`。

H7Ultra 的 `board_id=1179` 可能与其他板复用；**绝不能仅以 board_id 判断刷机安全**。保留原厂固件和参数备份。

## 5. 历史脚本说明

早期的 `apply_h7ultra_board.py`、`install_vendor_bsp.py` 和 `apply_gs_link.py` 处理的是不同 PX4 基线和分步工作流，已经被 **`integrate_px4_v1160.py` 取代**。避免混用两套编译基线。

来源核对：[VENDOR_SOURCE.md](VENDOR_SOURCE.md) · [SOURCE_MANIFEST.md](SOURCE_MANIFEST.md)
