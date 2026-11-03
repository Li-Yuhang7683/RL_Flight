# 本地 MAVLink 数据与绘图工作台

独立于 QGC、CEKF 和仿真运行时。Python 负责串口/网络接收与 MAVLink 校验、
解码；浏览器按字段名称绘制曲线。运行时不访问 CDN，也不需要 Node 构建或 Torch。

## 启动

在仓库根目录执行（Python 3.10 或更新版本）：

```bash
python3 -m venv WebUI/.venv
WebUI/.venv/bin/pip install -r WebUI/requirements-mavlink.txt
WebUI/.venv/bin/python WebUI/mavlink_server.py
```

打开 **http://127.0.0.1:8081**。Windows 可使用
`WebUI\.venv\Scripts\python.exe` / `pip.exe`。已有虚拟环境时只需后两步。

先检查界面与曲线可运行：

```bash
WebUI/.venv/bin/python WebUI/mavlink_server.py --demo
```

演示数据有显式标记，经过真正的 MAVLink 2 编码与解析，不会连接飞控。
现有的仿真 `WebUI/server.py` 保持原入口；MAVLink 工作台使用独立的 8081 端口。

如需另一台电脑的浏览器访问，可增加 `--host 0.0.0.0 --port 8081`，然后打开
`http://服务电脑IP:8081`。服务不带账号认证，只应监听可信的本地网络。
浏览器控制的是 **运行 Python 服务的电脑** 上的串口，不是浏览器所在电脑的串口。

## 连接飞控

页面支持：

| 方式 | 输入示例 | 说明 |
| --- | --- | --- |
| 串口 / USB | `/dev/ttyACM0`、`/dev/ttyUSB0`、`COM3` | 选择匹配波特率；下拉建议来自本机串口枚举 |
| UDP 监听 | `0.0.0.0:14550` | 让飞控/SITL/路由器将 MAVLink 发到这台电脑的 IP 和此端口 |
| UDP 远端 | `192.168.1.10:14550` | 主动向远端发送 GCS 心跳，以建立返回地址 |
| TCP 客户端 | `127.0.0.1:5760` | 连接已有的 MAVLink TCP 服务 |

同一串口不能同时由 QGC 和本服务打开。同一 UDP 监听端口也请只分配给一个程序；
需要多程序时，让 mavlink-router 分发到不同端口。

默认发送 1 Hz GCS 心跳，可在页面关闭。工具只接收遥测，不发送解锁、模式或执行器
控制命令。点击“请求 IMU / CEKF 输出”时，才额外向指定系统/组件发送
`MAV_CMD_SET_MESSAGE_INTERVAL`，请求消息 251（NAMED_VALUE_FLOAT）的输出频率。
页面会显示命令 ACK。自动填充的目标来自飞控 HEARTBEAT，也可手动修改。

**连接成功只表示设备/端口已打开**；请结合接收速率、最近接收时间、错误提示确认
是否有数据。断流后，最近接收时间会增长，字段标为过期；NaN 作为无效值显示，
曲线中断，不会变成 0。连接错误需要修正地址后重新连接，不会静默换到演示数据。

## 双机 IMU 字段

需要刷入前面包含 `COMPOSITE_IMU` 流的固件，并正确设置 Airlink 主从角色。
连接 **主机**，可以收到主机本地 IMU 与通过 Airlink 传来的从机 IMU。
连接从机只能看到从机本地通道，其余通道通常为 NaN。
不需要启动 `comp_attitude`，也不需要设置 `CEKF_EN=1`。

| 名称，末尾分别为 X/Y/Z | 意义 | 单位 |
| --- | --- | --- |
| `A_ACC_R_X` / `Y` / `Z` | 主机原始加速度 | m/s² |
| `B_ACC_R_X` / `Y` / `Z` | 从机原始加速度 | m/s² |
| `A_ACC_C_X` / `Y` / `Z` | 主机映射到 C 系的加速度 | m/s² |
| `B_ACC_C_X` / `Y` / `Z` | 从机映射到 C 系的加速度 | m/s² |
| `A_GYR_R_X` / `Y` / `Z` | 主机原始角速度 | rad/s |
| `B_GYR_R_X` / `Y` / `Z` | 从机原始角速度 | rad/s |
| `A_GYR_C_X` / `Y` / `Z` | 主机映射到 C 系的角速度 | rad/s |
| `B_GYR_C_X` / `Y` / `Z` | 从机映射到 C 系的角速度 | rad/s |

R 是坐标映射前的 SI 值，不是 ADC 计数。C 系值采用固件提供的结果；网页不使用
估计姿态做二次旋转。安装角取决于固件参数 `CEKF_MSTR_YAW` / `CEKF_SLV_YAW`。
本版参数直接表示 FRD 向量映射角：A→C 为 -30°，B→C 为 +30°，
分别采用 `Rz_FRD(-30°)` 和 `Rz_FRD(+30°)`；C 前向轴在主机右侧 30°。
加速度计字段为比力，水平静止时 Z 约为 -g。
本机直接观察的 R 通道仍来自驱动层 `sensor_accel/sensor_gyro`，C 通道仅加上述 yaw，
不等同于 CEKF 的本机修正后输入。新固件的 Airlink 从机转发通道则来自选中的
`vehicle_imu`，已经包含 PX4 标定、配置的温度补偿和板到机体系旋转。
因此连接主机时，A 通道是驱动层数据，B 通道是从机修正后的数据；
`C_ACC_*` / `C_GYR_*` 为 CEKF 实际融合输入。比较 A/B 差值时需考虑这一区别。
旧固件中的 `A_ACCR_X` 等名称自动规范为 `A_ACC_R_X`。
旧 `DEBUG_FLOAT_ARRAY`（`IMU_AB_C`）和 `DEBUG_VECT` 的 IMU 字段也可以解析。

除了 IMU，标准 common dialect MAVLink 消息的数值字段、数值数组也会自动出现，
例如 `ATTITUDE.roll`。来源使用 `system/component/message/name` 唯一区分，
不同飞机、组件或消息的同名数据不会互相覆盖。

## CEKF 状态和融合曲线

刷入最新 `micoair_h743-v2_nncontrol.px4`，连接主机。在 PX4 控制台执行：

```sh
param set CEKF_EN 1
comp_attitude start
comp_attitude status
```

在工作台选择 **预设：CEKF 融合** 或 **预设：CEKF 状态**；左侧 **CEKF** 可筛选全部
27 个新增字段。也可在 QGC 的 MAVLink Inspector 中展开 `NAMED_VALUE_FLOAT`，选择对应名称绘图。
这些字段直接来自独立 CEKF 输出，不会替代两机的标准 `ATTITUDE` 遥测。

| 名称 | 含义 | 单位/取值 |
| --- | --- | --- |
| `C_ROLL` / `C_PITCH` / `C_YAW` | 组合体姿态 | deg，yaw 为 -180～180° |
| `C_ACC_X` / `Y` / `Z` | CEKF 实际使用的 C 系虚拟 IMU 比力，含启用的杆臂补偿 | m/s² |
| `C_GYR_X` / `Y` / `Z` | CEKF 实际使用的 C 系虚拟 IMU 角速度 | rad/s |
| `C_ALP_X` / `Y` / `Z` | C 系角加速度 | rad/s² |
| `CE_RUN` | 模块状态心跳新鲜 | 1 运行，0 停止或超过 200 ms 未更新 |
| `CE_EN` / `CE_INIT` / `CE_VALID` | 启用 / 初始化完成 / 融合输出有效 | 0/1 |
| `CE_STATE` | 估计器模式 | 0 初始化，1 双机融合，2 主机单独更新，3 故障 |
| `CE_LINK` | CEKF 使用的 Airlink 连接状态 | 0/1 |
| `CE_W_A` / `CE_W_B` | 主机 / 从机 IMU 权重 | 连接 0.5/0.5，断开 1/0 |
| `CE_RESET` | CEKF 重置计数 | uint8，溢出回绕 |
| `CE_FLAGS` | 融合健康标志位 | 位掩码，与 uORB 输出一致 |
| `CE_AGE_A` / `CE_AGE_B` | 输入相对本机处理时刻的年龄 | ms；从机使用接收时间，断链时 B 为 0 |
| `CE_FRAMES` | 双机融合样本累计数 | 计数，经 float 传输，大计数有精度限制 |
| `CE_YAWREF` | 下发的 yaw 参考 | deg |
| `CE_YSIG` | yaw 参考标准差 | rad |

当 `CEKF_EN=0` 或缺少初始化输入时，模块仍持续发布状态，便于区别“模块已启动”和
“融合已就绪”。停止模块后 `CE_RUN=0`、`CE_VALID=0`，其余过期数据发为 NaN；
融合数据超过 200 ms 未刷新也会断开曲线，避免把缓存误认为持续更新。

沿用 `COMPOSITE_IMU` 流和消息 251，每轮共 51 个具名标量。
Normal、Onboard、Config 模式默认目标 10 Hz，实际频率受 MAVLink 带宽分配影响。
51 个 MAVLink 2 无签名包在 10 Hz 时约占 15.3 kB/s；115200 串口可降至 5 Hz，
USB 可保留 10 Hz。可以点击页面“请求 IMU / CEKF 输出”，或在 PX4 控制台为实际端口设置：

```sh
mavlink status
mavlink stream -d /dev/ttyACM0 -s COMPOSITE_IMU -r 10
```

这里的频率是整组标量的目标刷新频率，模块本身的 IMU 更新频率保持不变。
页面演示模式也包含 CEKF 字段，每 8 秒模拟连接/断开一次，便于预览权重切换。

## 自由绘图

1. 使用双机 IMU、“CEKF 融合”或“CEKF 状态”预设快速开始。
2. 点击任意图表，将其设为当前图；左侧勾选字段，即可把任意字段叠加到这个图。
3. “添加图表”创建空图；标题可编辑；图例中 `×` 单独移除曲线。
4. 每条曲线可选左轴/右轴；不同单位会优先放到另一轴。左右轴分别自动缩放。
   “纵轴”可填写固定范围；“自动”恢复自动缩放。多种单位手动放同一轴时共用刻度。
5. 时间窗支持 5 秒到 5 分钟；滚轮沿指针位置缩放，拖动平移；双击或“回到实时”恢复跟随。
6. “暂停查看”冻结视窗，后台仍继续接收。指针悬停显示该时间附近各曲线的值。
7. “CSV”导出**当前可见时间窗、所有图表已选择字段**的采样点，采用长表格式，
   包含 UTC 接收时间、系统、组件、消息、字段名、单位和数值。无效值导出为 `NaN`。
8. 图表布局保存在当前浏览器的 localStorage，刷新后保留；遥测数据不落盘，连接新源
   会重置数据缓存，浏览器布局中的字段选择保留。

时间轴使用 **Python 接收到消息的时间**，不对两机时钟或传输延迟做同步校正；
可以检查轴、方向与趋势，不能据此声称两机采样严格同步。图中原始值为实线，C 系值
为虚线。缓存按最近 300 秒、最多 120,000 个标量采样点保留，以先触及的限制为准；
最多发现 2,048 个字段。暂停超过保留时长后，较早曲线会被淘汰。
浏览器曲线按像素压缩并保留桶内极值，CSV 使用压缩前的缓存采样。

## 验证

```bash
WebUI/.venv/bin/python -m unittest WebUI.tests.test_mavlink_telemetry -v
node WebUI/tests/test_mavlink_plot.mjs
```

覆盖 MAVLink 1/2 分包、24 个原始 IMU 通道和 27 个 CEKF 通道身份、旧格式、NaN、损坏帧恢复、缓存游标、
串口伪终端、真实本地 UDP/TCP、HTTP 生命周期与跨源请求限制。
物理飞控、Airlink 链路和现场安装角仍需用实际设备确认。

浏览器交互检查可另外安装 Playwright（仅用于测试），在 `--demo` 服务运行时执行：

```bash
WebUI/.venv/bin/python WebUI/tests/check_mavlink_browser.py
```

可通过 `--chromium /path/to/chrome` 使用已有 Chromium。测试覆盖选字段、独立图表、
左右轴与范围、暂停、缩放、CSV、布局恢复及手机宽度；截图和导出文件写到临时目录。
