# Ryan PX4 与官方 PX4 的全量变更审计（2026-11-11）

> **完整性边界**：本报告列举固定版本之间的全部 **105 个 Ryan 自有变更文件**及官方 **44 个维护提交涉及的 51 个文件**。源代码文件的用途按实际变更和相关源码梳理；GitHub 不提供二进制内部补丁，因此对 .bin/.zip/.px4 文件不推断内部功能。它是源码树级差异清单，不等于对每个数字增益的验证或功能/飞行认证。
>
> Ryan 固定提交：`3fef8242687cf733b590ff21c5b36779943299d1`
> 官方基线 v1.16.2：`54f0455ffcd755534539a7cf33a09a20bf71d29d`
> H7Ultra 原厂对应 v1.16.0：`6ea3539157ca358c70a515878b77077af7d4611d`

## 一、三段对比的正确拆分

| 起点 → 终点 | Git 提交 | 涉及文件 | 新增行 | 删除行 |
| --- | ---: | ---: | ---: | ---: |
| 官方 1.16.2 → Ryan 固定版 | 11 | 105 | 16264 | 37 |
| 官方 1.16.0 → 官方 1.16.2 | 44 | 51 | 605 | 236 |
| 官方 1.16.0 → Ryan 固定版 | 55 | 154（去重） | 16,867 | 271 |

首两段文件集合重叠 2 项：`platforms/nuttx/src/px4/common/SerialImpl.cpp` 和 `src/modules/ekf2/EKF/ekf.h`。**不要将 1.16.0→1.16.2 的 44 个官方修复提交归为 Ryan 独创。**

[Ryan 自有完整对比](https://github.com/Ryan-5853/PX4-TeleaiTrans/compare/PX4:v1.16.2...3fef8242687cf733b590ff21c5b36779943299d1) / [官方维护差异](https://github.com/PX4/PX4-Autopilot/compare/v1.16.0...v1.16.2) / [H7Ultra 1.16.0 对比 Ryan](https://github.com/Ryan-5853/PX4-TeleaiTrans/compare/PX4:v1.16.0...3fef8242687cf733b590ff21c5b36779943299d1)

## 二、实际改变系统行为的高优先级事项

1. **专用直接控制目标**：`micoair_h743-v2_nncontrol` 关掉 stock 多旋翼位置/姿态/角速度控制、`control_allocator`、flight_mode_manager；`nn_control` 直接发布 2 motor + 3 servo uORB 输出。MicoAir 默认 target 未必采用这一模式。
2. **LQI**：后端 0=标称 13 状态、1=辨识组合 19 状态、2=共轴 collective/servo；模型和物理机体编号不符禁止执行器输出。`NN_LQI_OUTPUT_EN=0` 默认禁用，显式启用才可能通过后端门禁。
3. **量化 NN 不等于可飞网络**：`NeuralControllerBackend.cpp` 明确为 21-128-128-4 int8 GRU 随机权重验证，`actuatorOutputAllowed()` 返回 false。Hybrid 也是空壳。
4. **RC 监督 GS**：只有物理 RC 有效、已 armed、正确模式槽、GS 数据新鲜、上锁后重新切换模式等才允许 GS 四轴代替人工轴；失链回退 RC 并要求重新切换。GS-Link 通过 UART1 4M 与 ESP 连接，不是 MAVLink。
5. **Airlink**：新增 6M 波特率双机串口、角色、会话握手、配对上锁、传感器和执行器协调、断链处理。不是 GS-Link 的别名。
6. **Composite EKF**：独立组合 IMU 姿态估计，扩展 EKF2 允许外部初始化/重置姿态，MAVLink 51 路 NAMED_VALUE_FLOAT 调试数据。
7. **全局 Commander 行为变化**：取消 RC 开关重新解锁 5 秒内绕过 preflight 检查的宽限；标准目标编入该源码也会受到此改动影响。
8. **Logger 全局修改**：不再优先采用原 logger_topics.txt/profile 逻辑，而固定采用辨识主题；核心主题间隔是 **5ms=200Hz**；同时 logger 运行间隔封顶 **2ms=500Hz**。日志输出文本出现“500Hz core”不代表每个核心主题实际都以 500Hz 记录。
9. **HIL 改造**：HIL_ACTUATOR_CONTROLS 默认从 200Hz 提高至 500Hz，copy 最新输出、恒定流率；修正 HIL_SENSOR 加速度单位；HIL armed 时继续扫描模拟 IMU；PWM Sim 增加诊断。
10. **MAVLink 改动**：支持 4M 串口，新增 COMPOSITE_IMU 默认流（51 名称值/帧，10Hz），会改变该模式下的遥测带宽，和标准 PX4 的行为不同。
11. **MicoAir 独有板级**：USART2 RX 缓冲扩大 600→4096，GPIO reset 输出低，SD buffer=256KiB、1000Hz logging args；增加 22001 机型和硬件输出映射。
12. **二进制变化**：有 MicoAir bootloader 修改，以及打包固件/Windows 工具；不是 H7Ultra 固件，也没有本报告能验证的刷机安全证明。

## 三、Ryan 全部 105 个变更文件（按子系统分类）

状态字段：新增=added，修改=modified；增/删是 GitHub 比较器报告的文本行数。

### 启动、机型与 ROMFS（6 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `ROMFS/px4fmu_common/init.d/CMakeLists.txt` | +7/−0 | NN 控制器选中时打包 direct_control 专用启动脚本 |
| 新增 | `ROMFS/px4fmu_common/init.d/airframes/22001_direct_actuator_controller` | +85/−0 | 新增 22001 共轴双电机三舵机机型；设置 GS-Link、Airlink、RC 模式与 PWM 默认值 |
| 修改 | `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` | +7/−0 | 将 22001 机型脚本纳入 ROMFS（仅 NN_CONTROL 构建） |
| 新增 | `ROMFS/px4fmu_common/init.d/rc.direct_control_apps` | +19/−0 | 启动 land_detector、GS-Link(4M)、Airlink(6M)、nn_control，不启常规控制器 |
| 新增 | `ROMFS/px4fmu_common/init.d/rc.direct_control_defaults` | +13/−0 | 设 VEHICLE_TYPE=direct_control、MAV_TYPE=4、陀螺仪速率上限 |
| 修改 | `ROMFS/px4fmu_common/init.d/rc.vehicle_setup` | +8/−0 | 增加 direct_control 机型启动分支 |

### 工具与固件产物（11 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `Tools/module_config/generate_actuators_metadata.py` | +7/−8 | 允许无 Control Allocator 时生成执行器元数据 |
| 新增 | `Tools/nn_control/README_WINDOWS.md` | +211/−0 | Windows 上位机 GRU 验证操作手册 |
| 新增 | `Tools/nn_control/check_environment.bat` | +22/−0 | Windows Python/工具依赖预检查脚本 |
| 新增 | `Tools/nn_control/firmware/FIRMWARE_INFO.txt` | +12/−0 | 打包固件版本与核验信息说明 |
| 新增 | `Tools/nn_control/firmware/micoair_h743-v2_nncontrol.px4` | +19/−0 | 包含在仓库的 MicoAir 测试固件成品（非 H7Ultra） |
| 新增 | `Tools/nn_control/requirements-windows.txt` | +5/−0 | Windows 验证所需 Python 依赖 |
| 新增 | `Tools/nn_control/run_verification.bat` | +47/−0 | 启动 Windows 验证流程 |
| 新增 | `Tools/nn_control/setup_windows.bat` | +53/−0 | 安装 Windows 验证依赖 |
| 新增 | `Tools/nn_control/verify_gru_mavlink.py` | +708/−0 | GRU 计算对比/时延验证与 MAVLink 测试通信 |
| 新增 | `Tools/nn_control_windows_bundle.zip` | +0/−0 | Windows 验证工具打包压缩文件（二进制） |
| 修改 | `Tools/serial/serial_params.c.jinja` | +1/−0 | 串口参数可选 4,000,000 baud |

### MicoAir 板级硬件（5 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `boards/micoair/h743-v2/extras/micoair_h743-v2_bootloader.bin` | +0/−0 | 改动 MicoAir 引导加载程序二进制；比较器无法解释内部字节 |
| 修改 | `boards/micoair/h743-v2/init/rc.board_defaults` | +32/−1 | SD 日志缓冲/1000Hz、HIL 输出顺序、22001 机型、DShot/电池/模式默认参数 |
| 新增 | `boards/micoair/h743-v2/nncontrol.px4board` | +88/−0 | 定制 NN 控制目标：启 NN_CONTROL/GS_LINK/AIRLINK/COMP_ATTITUDE，关官方 MC 控制与控制分配 |
| 修改 | `boards/micoair/h743-v2/nuttx-config/nsh/defconfig` | +1/−1 | USART2 RX 缓冲从 600 增至 4096 字节 |
| 修改 | `boards/micoair/h743-v2/src/init.c` | +2/−1 | 飞控复位期间将 PWM/DShot 输出 GPIO 配置为低电平输出 |

### 文档（4 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `docs/image.png` | +0/−0 | 新增文档图片（二进制，无法从比较文本判断内容） |
| 修改 | `docs/zh/SUMMARY.md` | +1/−0 | 在中文文档目录加入 nn_control 稳定单体控制文档链接 |
| 新增 | `docs/zh/advanced/nn_control_coaxial_collective_baseline.md` | +77/−0 | 共轴 collective/servo LQI 基线说明 |
| 新增 | `docs/zh/advanced/nn_control_gru_500hz.md` | +263/−0 | 量化 GRU 500Hz 验证说明 |

### uORB 消息定义（12 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `msg/AirlinkActuatorCommand.msg` | +10/−0 | 双机执行器指令消息 |
| 新增 | `msg/AirlinkPairCommand.msg` | +12/−0 | 双机配对与会话命令消息 |
| 新增 | `msg/AirlinkSensorState.msg` | +20/−0 | Airlink 远端传感器状态消息 |
| 新增 | `msg/AirlinkStatus.msg` | +55/−0 | 双机通信/配对状态消息 |
| 新增 | `msg/AirlinkTestResult.msg` | +33/−0 | Airlink 链路测试结果消息 |
| 新增 | `msg/AirlinkYawReference.msg` | +10/−0 | Airlink 远端 yaw 参考消息 |
| 修改 | `msg/CMakeLists.txt` | +11/−0 | 注册 11 个新增 uORB 消息主题定义 |
| 新增 | `msg/CompositeAttitudeEstimate.msg` | +21/−0 | 组合体姿态/动力学估计消息 |
| 新增 | `msg/CompositeEstimatorStatus.msg` | +74/−0 | 组合体估计器状态/融合健康信息 |
| 新增 | `msg/CompositeYawReference.msg` | +15/−0 | 组合体 yaw 参考消息 |
| 新增 | `msg/GsLinkControl.msg` | +14/−0 | GS-Link 控制轴和心跳状态消息 |
| 新增 | `msg/GsLinkStatus.msg` | +13/−0 | GS-Link 门控、安全与模式状态消息 |

### PX4 平台与串口（2 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp` | +6/−0 | 新增 AIRLINK 专用工作队列，8192 栈、-14 优先级 |
| 修改 | `platforms/nuttx/src/px4/common/SerialImpl.cpp` | +14/−2 | 支持 B4000000；串口 write 错误 errno 保留与日志改进 |

### Airlink 双机链路（9 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `src/modules/airlink/Airlink.cpp` | +1534/−0 | Airlink 飞控模块：6M 串口通信、收发、配对、传感器交换及安全状态 |
| 新增 | `src/modules/airlink/Airlink.hpp` | +232/−0 | Airlink 类/接口及状态字段声明 |
| 新增 | `src/modules/airlink/AirlinkImu.hpp` | +111/−0 | Airlink IMU 数据处理/封装辅助 |
| 新增 | `src/modules/airlink/AirlinkProtocol.hpp` | +1364/−0 | Airlink 专用二进制帧与协议类型/校验 |
| 新增 | `src/modules/airlink/AirlinkSession.cpp` | +2112/−0 | 双机配对、会话状态、ACK 重试、链路中断与恢复状态机 |
| 新增 | `src/modules/airlink/AirlinkSession.hpp` | +305/−0 | Airlink 会话管理数据结构与接口 |
| 新增 | `src/modules/airlink/CMakeLists.txt` | +21/−0 | Airlink PX4 模块构建定义 |
| 新增 | `src/modules/airlink/Kconfig` | +7/−0 | Airlink 可选模块开关 |
| 新增 | `src/modules/airlink/airlink_params.c` | +102/−0 | AIRLINK_ROLE/PAIR_EN、着陆冲击/翻转/超时等参数 |

### Commander 安全（1 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/commander/Commander.cpp` | +6/−6 | 取消 RC 开关解锁后 5s 内可跳过 preflight 的重新解锁宽限 |

### 复合姿态估计（6 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `src/modules/comp_attitude/CMakeLists.txt` | +36/−0 | 组合姿态估计 PX4 模块构建定义 |
| 新增 | `src/modules/comp_attitude/CompAttitude.cpp` | +462/−0 | 双飞行器 IMU 组合姿态估计运行实现 |
| 新增 | `src/modules/comp_attitude/CompAttitude.hpp` | +162/−0 | 组合估计器状态/订阅/发布接口 |
| 新增 | `src/modules/comp_attitude/CompositeMath.hpp` | +562/−0 | 组合体坐标转换、IMU 融合和相关数学工具 |
| 新增 | `src/modules/comp_attitude/Kconfig` | +10/−0 | 需依赖 Airlink+EKF2 的组合姿态模块开关 |
| 新增 | `src/modules/comp_attitude/module.yaml` | +172/−0 | CEKF 多项融合、姿态、传感器映射与杆臂参数 |

### EKF2（4 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/ekf2/CMakeLists.txt` | +5/−0 | 启 COMP_ATTITUDE 时将复合估计模块参与 EKF2 构建 |
| 修改 | `src/modules/ekf2/EKF/ekf.h` | +28/−0 | 增加外部 yaw/四元数重置及独立 EKF 姿态初始化 API |
| 修改 | `src/modules/ekf2/EKF/yaw_fusion.cpp` | +43/−0 | 实现外部姿态重置：协方差、输出预测器、重置计数与 tilt/yaw align |
| 修改 | `src/modules/ekf2/test/test_EKF_initialization.cpp` | +30/−0 | 增加外部姿态重置与运动中直接初始化单元测试 |

### GS-Link 地面通信（5 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `src/modules/gs_link/CMakeLists.txt` | +18/−0 | GS-Link PX4 模块编译配置 |
| 新增 | `src/modules/gs_link/GsLink.cpp` | +779/−0 | GS-Link 串口收发、CRC 解析、遥测 50Hz、心跳 20Hz、指令 ACK |
| 新增 | `src/modules/gs_link/GsLink.hpp` | +124/−0 | GS-Link 接口、uORB 订阅/发布与状态 |
| 新增 | `src/modules/gs_link/GsLinkProtocol.hpp` | +212/−0 | GS-Link v1 帧格式、消息 ID、定长数据结构和 CRC16 |
| 新增 | `src/modules/gs_link/Kconfig` | +6/−0 | GS-Link 可选模块开关 |

### ULog/logger（2 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/logger/logged_topics.cpp` | +40/−8 | 将 profile/文件选择逻辑改为固定辨识主题表；5ms 核心主题=200Hz |
| 修改 | `src/modules/logger/logger.cpp` | +9/−0 | _rate_factor 强制 1；日志循环间隔上限 2ms(500Hz) |

### MAVLink/HIL（5 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/mavlink/mavlink_main.cpp` | +12/−4 | 增加 4M 波特率、HIL_ACTUATOR_CONTROLS 500Hz 与 10Hz COMPOSITE_IMU 默认流 |
| 修改 | `src/modules/mavlink/mavlink_messages.cpp` | +4/−0 | 注册 COMPOSITE_IMU MAVLink 流类型 |
| 修改 | `src/modules/mavlink/mavlink_receiver.cpp` | +8/−0 | HIL_SENSOR 接收时恢复加速度 SI 单位缩放比例 |
| 新增 | `src/modules/mavlink/streams/COMPOSITE_IMU.hpp` | +237/−0 | 新增 51 路 NAMED_VALUE_FLOAT 复合 IMU/融合状态可视化 |
| 修改 | `src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp` | +10/−1 | HIL 输出流恒定速率并 copy 最新执行器输出 |

### Navigator 配置（1 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/navigator/Kconfig` | +1/−1 | NUM_MISSION_ITMES_SUPPORTED 的 Kconfig 依赖从 NAVIGATOR 改为 DATAMAN |

### nn_control 和 LQI/Neural（27 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 新增 | `src/modules/nn_control/AirlinkLandingGuard.hpp` | +78/−0 | Airlink 断链后单机降落/冲击与翻转监测逻辑 |
| 新增 | `src/modules/nn_control/CMakeLists.txt` | +41/−0 | nn_control 的三种后端编译选择与模块链接 |
| 新增 | `src/modules/nn_control/ControllerBackend.hpp` | +80/−0 | 统一 Neural/LQR/Hybrid 后端接口 |
| 新增 | `src/modules/nn_control/ControllerTypes.hpp` | +61/−0 | ControllerInput、2 电机 3 舵机规范化接口 |
| 新增 | `src/modules/nn_control/FinalMotorOutputMap.hpp` | +29/−0 | 执行器边界统一给电机命令乘 0.5 标定比例 |
| 新增 | `src/modules/nn_control/GroundStationGate.hpp` | +173/−0 | 物理 RC 监督、自定义外部模式及 GS 失链回退门控 |
| 新增 | `src/modules/nn_control/HybridControllerBackend.cpp` | +94/−0 | Hybrid 后端空壳，主动拒绝执行器输出 |
| 新增 | `src/modules/nn_control/Kconfig` | +37/−0 | nn_control 开关、Neural/LQR/Hybrid 互斥后端 |
| 新增 | `src/modules/nn_control/LqiCoaxialCollectiveCore.hpp` | +449/−0 | 共轴 collective/servo LQI 计算核心 |
| 新增 | `src/modules/nn_control/LqiCoaxialCollectiveModel.hpp` | +93/−0 | 共轴 LQI 模型规格与数据结构 |
| 新增 | `src/modules/nn_control/LqiCoaxialCollectivePayload.hpp` | +216/−0 | 共轴 LQI 的固定载荷、标识与辨识溯源 |
| 新增 | `src/modules/nn_control/LqiCompositeControllerModel.hpp` | +84/−0 | 19 状态组合 LQI 模型规格 |
| 新增 | `src/modules/nn_control/LqiCompositeCore.hpp` | +302/−0 | 19 状态组合 LQI 运行核心 |
| 新增 | `src/modules/nn_control/LqiControllerCore.hpp` | +324/−0 | 基础 13 状态 LQI 核心 |
| 新增 | `src/modules/nn_control/LqiControllerModel.hpp` | +63/−0 | 基础 LQI 模型规格 |
| 新增 | `src/modules/nn_control/LqiIdentifiedModel.hpp` | +209/−0 | 辨识后的组合 LQI 模型数据/增益 |
| 新增 | `src/modules/nn_control/LqiManualReference.hpp` | +133/−0 | RC 轴指令到 LQI 目标姿态/状态转换 |
| 新增 | `src/modules/nn_control/LqiNominalModel.hpp` | +146/−0 | 标称 LQI 模型/增益与参数 |
| 新增 | `src/modules/nn_control/LqrControllerBackend.cpp` | +329/−0 | 选择标称13状态、辨识19状态或共轴 LQI 及机体匹配/输出许可 |
| 新增 | `src/modules/nn_control/NNControl.cpp` | +1456/−0 | 500Hz 直接控制任务、RC/GS 切换、failsafe、actuator 发布 |
| 新增 | `src/modules/nn_control/NNControl.hpp` | +243/−0 | nn_control 订阅、参数、性能和状态定义 |
| 新增 | `src/modules/nn_control/NNVerificationProtocol.hpp` | +73/−0 | NN/LQI 验证调试消息协议 |
| 新增 | `src/modules/nn_control/NeuralControllerBackend.cpp` | +129/−0 | 21-128-128-4 int8 GRU 试验后端，随机权重，不给执行器权限 |
| 新增 | `src/modules/nn_control/NeuralNetworkInterface.cpp` | +355/−0 | int8 GRU 前向、量化、权重初始化和核验 |
| 新增 | `src/modules/nn_control/NeuralNetworkInterface.hpp` | +74/−0 | GRU 网络结构、接口与静态维度 |
| 新增 | `src/modules/nn_control/module.yaml` | +40/−0 | QGC 机型执行器元数据与 2 电机/3 舵机映射 |
| 新增 | `src/modules/nn_control/nn_control_params.c` | +175/−0 | NN_* 启用、LQI 模型/机体 ID、GS 安全门控与舵机零偏 |

### 遥控 RC（2 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/rc_update/rc_update.cpp` | +3/−2 | manual_control_switches 状态周期发布从约 1Hz 调到 10Hz |
| 修改 | `src/modules/rc_update/rc_update.h` | +1/−0 | 增加 100ms 频率常量 |

### 传感器/HIL（1 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/sensors/sensors.cpp` | +8/−2 | HIL 模式 armed 状态下仍允许新模拟 IMU 被扫描/注册 |

### 仿真输出（2 个）

| 状态 | 文件 | + / − | 主要作用与差异 |
| --- | --- | ---: | --- |
| 修改 | `src/modules/simulation/pwm_out_sim/PWMSim.cpp` | +5/−0 | 增加 HIL PWM publication 计数和运行周期性能统计 |
| 修改 | `src/modules/simulation/pwm_out_sim/PWMSim.hpp` | +1/−0 | 增加 publication counter 字段 |

## 四、官方 1.16.0 → 1.16.2 的全部 51 个维护文件（这不是 Ryan 的专有改动）

官方这 44 个提交大多是 v1.16 分支的发布/CI/文档维护、多个其他机型板级驱动及 MAVLink/EKF/Navigator/Serial 等修复。对 H7Ultra 的影响应单独基于硬件目标验证，不能把这些称作 GS-Link 功能。

| 文件 | + / − | 状态 |
| --- | ---: | --- |
| `.github/workflows/compile_macos.yml` | +5/−2 | modified |
| `.github/workflows/docs_deploy.yml` | +0/−12 | modified |
| `.github/workflows/docs_deploy_aws.yml` | +114/−0 | added |
| `.github/workflows/failsafe_sim.yml` | +1/−0 | modified |
| `ROMFS/px4fmu_common/init.d-posix/rcS` | +4/−1 | modified |
| `Tools/setup/macos.sh` | +2/−0 | modified |
| `boards/ark/fpv/default.px4board` | +2/−0 | modified |
| `boards/ark/fpv/encrypted_logs.px4board` | +1/−0 | modified |
| `boards/cuav/7-nano/default.px4board` | +1/−0 | modified |
| `boards/cuav/7-nano/init/rc.board_sensors` | +9/−2 | modified |
| `boards/cuav/7-nano/src/spi.cpp` | +1/−0 | modified |
| `boards/modalai/voxl2/target/voxl-px4-start` | +9/−2 | modified |
| `boards/px4/fmu-v6xrt/default.px4board` | +2/−0 | modified |
| `boards/px4/fmu-v6xrt/init/rc.board_sensors` | +33/−9 | modified |
| `boards/px4/fmu-v6xrt/src/board_config.h` | +2/−1 | modified |
| `boards/px4/fmu-v6xrt/src/spi.cpp` | +20/−0 | modified |
| `docs/.vitepress/config.mjs` | +7/−3 | modified |
| `docs/en/SUMMARY.md` | +4/−9 | modified |
| `docs/en/_sidebar.md` | +14/−13 | modified |
| `docs/en/actuators/index.md` | +1/−2 | modified |
| `docs/en/index.md` | +1/−10 | modified |
| `docs/en/releases/1.15.md` | +0/−2 | modified |
| `docs/en/releases/1.16.md` | +193/−0 | added |
| `docs/en/releases/index.md` | +2/−1 | modified |
| `docs/en/releases/main.md` | +2/−130 | modified |
| `docs/ko/_sidebar.md` | +12/−1 | modified |
| `docs/uk/_sidebar.md` | +8/−1 | modified |
| `docs/zh/_sidebar.md` | +12/−1 | modified |
| `platforms/common/Serial.cpp` | +5/−0 | modified |
| `platforms/common/include/px4_platform_common/Serial.hpp` | +1/−0 | modified |
| `platforms/nuttx/src/px4/common/SerialImpl.cpp` | +17/−2 | modified |
| `platforms/nuttx/src/px4/common/include/SerialImpl.hpp` | +1/−0 | modified |
| `platforms/posix/include/SerialImpl.hpp` | +1/−0 | modified |
| `platforms/posix/src/px4/common/SerialImpl.cpp` | +17/−3 | modified |
| `platforms/posix/src/px4/common/px4_daemon/pxh.cpp` | +4/−2 | modified |
| `platforms/qurt/include/SerialImpl.hpp` | +1/−0 | modified |
| `platforms/qurt/src/px4/SerialImpl.cpp` | +7/−0 | modified |
| `src/drivers/gps/gps.cpp` | +22/−4 | modified |
| `src/drivers/uavcan/actuators/esc.cpp` | +1/−5 | modified |
| `src/modules/commander/accelerometer_calibration.cpp` | +10/−3 | modified |
| `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp` | +1/−1 | modified |
| `src/modules/ekf2/EKF/covariance.cpp` | +3/−1 | modified |
| `src/modules/ekf2/EKF/ekf.h` | +4/−1 | modified |
| `src/modules/ekf2/EKF/ekf_helper.cpp` | +5/−2 | modified |
| `src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp` | +2/−2 | modified |
| `src/modules/mavlink/mavlink_parameters.cpp` | +3/−0 | modified |
| `src/modules/mavlink/streams/AVAILABLE_MODES.hpp` | +17/−2 | modified |
| `src/modules/navigator/mission.cpp` | +4/−0 | modified |
| `src/modules/navigator/mission_base.cpp` | +3/−1 | modified |
| `src/modules/navigator/rtl_direct_mission_land.cpp` | +7/−2 | modified |
| `src/modules/navigator/rtl_mission_fast.cpp` | +7/−3 | modified |

## 五、11 个 Ryan 提交（按历史顺序）

| 提交 | 日期（UTC） | 第一行提交信息 |
| --- | --- | --- |
| [`fdd62636d`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/fdd62636d32d5f06ad52986103d707467265e8f1) | 2026-07-31 | 26.7.31 验证nn部署可行性；将nn嵌入px4飞控并成功运行、验证。21-128-128-4GRU神经网络在8bit量化下约800us，算力充裕，并经过上位机比对，结果正确 |
| [`9d01c9adf`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/9d01c9adf94d6549fc1b8ad31258a59caf8cb904) | 2026-08-04 | 26.8.4 针对实机构型调整固件构型设置，执行器测试基本正常 |
| [`d129496f6`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/d129496f6643e8c96150eb7eec38a83ca27ded49) | 2026-08-05 | 26.8.5 为标称lqr控制器搭建推理矿建 |
| [`6c2b24adc`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/6c2b24adc7b84b22d5171dda69164bd96ba10bdc) | 2026-08-11 | 26.8.11 增加HIL相关适配，修复nuttx子仓库的小问题。上机测试没有问题，基本只剩控制器参数问题 |
| [`6b3b61c38`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/6b3b61c38a38d86ae8613592c583c45409c6529d) | 2026-08-11 | 26.8.11 锁定日志500hz |
| [`6f57277bd`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/6f57277bdc9bc3be22bf90c493eb467f4bc04122) | 2026-08-12 | 26.8.12 增加SD日志缓冲区设置，修复日志模块小问题。重构NN控制器状态管理；更新辨识后的权重，提升明显，能稳定悬停和简单动作 |
| [`b10322933`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/b103229335dc568d8ed41ea6b9e9bc8d898990d8) | 2026-08-29 | 更新辨识/标称LQR模型权重 |
| [`302510c5e`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/302510c5eb6a458248745557d1c2609aaf8887d7) | 2026-10-26 | feat(nn_control): 接入 RC 监督的地面站控制 |
| [`b00ffaff0`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/b00ffaff038c7e959d6868833e9c11175d521d18) | 2026-10-30 | feat(nn_control): establish stable single-airframe control baseline |
| [`bc308f0a1`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/bc308f0a1eb8eba82a55d89e8e89a53210ab302b) | 2026-11-02 | 26.11.2 增加airlink驱动。验证双向通信正常 |
| [`3fef82426`](https://github.com/Ryan-5853/PX4-TeleaiTrans/commit/3fef8242687cf733b590ff21c5b36779943299d1) | 2026-11-03 | 26.11.3 增加组合体姿态观测器comp_attitude，测试正常 |

## 六、对 H7Ultra 的移植决策建议

| 能力 | 第一阶段 GS-Link 通信 | 未来 LQI 控制 | 暂不移植 |
| --- | --- | --- | --- |
| GS-Link 5 文件 + 2 uORB 消息 | 需要 | 沿用 | — |
| H7Ultra BSP、USB MAVLink 和 UART1 互斥占用 | 需要 | 沿用 | 不复制 MicoAir BSP |
| nn_control（27 文件）、22001 机型、uORB 执行器输出 | 暂不要 | 完成硬件映射与安全验证后引入 | 不直接用于现有原厂控制 |
| Airlink（9 文件）、comp_attitude（6 文件）/ 双机 EKF | 不需要 | 只有双机组合体任务才需要 | 单机第一阶段不移植 |
| Commander/logger/mavlink/ekf2 的全球改动 | 不整体合并 | 逐项审查必要性 | 避免造成无关系统行为变化 |
| Neural/Hybrid 后端 | 不需要 | 与当前辨识 LQI 主线无关 | 保留不上飞机 |
| MicoAir bootloader、PWM、NuttX、rc.board_defaults | 不需要 | 用厂家 H7Ultra 板级实现 | **绝对不要照搬** |

### 审查边界

- 已精确遍历 GitHub compare 中全部 105 + 51 个变更文件，并对关键行为修改做源码追踪；不意味着对所有新增控制系数、二进制内部字节做了独立数值验证。
- GitHub compare 不提供 `.bin`/`.zip` 固件的二进制补丁，所以不能根据变更记录推断刷写风险已消除。
- 当前用户 fork 中 `feature/endlessflight-h7ultra-gs-link` 是官方 PX4 v1.16.0 加 H7Ultra BSP 的**移植准备方案**，不是这 105 个改动全部移植后的成品固件。
