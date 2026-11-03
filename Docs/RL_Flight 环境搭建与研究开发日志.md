# RL_Flight 环境搭建与神经网络辨识 LQR/LQI 研究日志

> **文档类型**：可持续更新的开发日志（环境快照 + 操作记录 + 问题复盘 + 研究计划）  
> **创建 / 最近整理**：2026-11-02  
> **项目**：[Li-Yuhang7683/RL_Flight](https://github.com/Li-Yuhang7683/RL_Flight)（Fork 自 [Ryan-5853/RL_Flight](https://github.com/Ryan-5853/RL_Flight)）  
> **当前阶段**：开发环境与 Train 基础测试完成；**已将研究范围锁定为神经网络系统辨识 + LQR/LQI，开始阅读源码**  
> **更新原则**：`✅ 已验证`、`🟡 已配置但未验证`、`⬜ 待完成`、`⛔ 当前阻塞` 必须分开记录。下文“曾验证”不代表未来升级后仍然成立。

---

## 0. 一页摘要（每次更新优先修改此处）

**当前核心目标**：理解并复现 `Identification/` 中的**离线神经网络系统辨识 → 等效动力学模型 → LQR/LQI 增益计算**实验；后续在公司提供的新飞机上，从飞行日志出发重新辨识，输出需经过评估的候选控制器参数。**不研究 PPO、SAC、端到端神经网络直接控制飞机**。

**当前已确认结果**：

- ✅ Windows 11 + WSL2 + Ubuntu 24.04.5，Ubuntu 文件系统安装于 D 盘。
- ✅ RTX 4060 Laptop GPU（约 8 GiB）可用于 WSL2 内的 PyTorch CUDA 计算。
- ✅ Miniforge / Conda 环境 `rl-flight` / Python 3.10.22。
- ✅ PyTorch `2.12.1+cu130`、TorchRL `0.13.3`、TensorDict `0.13.0`、PyYAML `6.0.2`；TensorDict GPU 张量、PPO/SAC 导入测试正常。
- ✅ 自己的 GitHub Fork 已建立并克隆到 WSL2；`origin`、`upstream` 配置正确；`gh` 已通过 HTTPS 登录。
- ✅ `rl-flight-train==0.1.0` 已可编辑安装；`pip check` 无冲突；`flight-train --help` 可用。
- ✅ `Train/tests/test_progress.py` 中 **3/3** 单元测试通过。
- ⛔ `simenv` Python 模块未安装，`TeleaiTrans_Simenv` 源码尚未取得；**现阶段不处理仿真**。
- ⬜ `Identification` / `Controller` 包的安装与相关单元测试尚未收到成功输出；**代码阅读无需等它们安装**。
- 🟡 Git 提交身份已设置；**尚无 `git push --dry-run` 或真正推送成功的确认输出**。

**下一步只做**：阅读第 4 节代码地图中的辨识主线，先理解数据格式、监督标签、模型结构、训练循环、状态空间与 Riccati 求解。不搭建 SimEnv、不跑端到端强化学习。

---

## 1. 系统、硬件和开发环境

> 以下版本为先前对话中出现的**历史快照**；若系统升级，请用末尾“环境重检”命令更新。

| 类别 | 配置 / 版本 | 状态与说明 |
|---|---|---|
| 宿主操作系统 | **Windows 11**；此前命令行报告版本 `10.0.26300.9711` | ✅ 历史快照；Windows 内核版本显示 `10.0...` 不等于 Windows 10 |
| 开发方式 | **Windows + WSL2**，已不再使用 Ubuntu 双系统作为主开发环境 | ✅ |
| WSL 组件版本 | `3.0.3.0` | ✅ 历史快照 |
| WSL Linux 内核 | `6.18.54.1-1` | ✅ 历史快照 |
| Linux 发行版 | **Ubuntu 24.04.5 LTS**（WSL 注册名 `Ubuntu-24.04`） | ✅ |
| Ubuntu 用户 | `lyh`；主目录 `/home/lyh` | ✅ |
| Linux 文件系统物理位置 | `D:\WSL\Ubuntu-24.04`（WSL 的虚拟磁盘） | ✅；`~/projects/...` 实际占用 D 盘，不是 C 盘 |
| 本地 GPU | NVIDIA **RTX 4060 Laptop GPU**，显存约 **8 GiB** | ✅ WSL 内 `nvidia-smi`、PyTorch GPU 计算验证 |
| NVIDIA 驱动/运行时显示 | 曾见 `nvidia-smi 615.78.02`、驱动 `617.14`、其显示 CUDA `13.4` | ✅ 仅历史输出；**不等同于** PyTorch Wheel 的 CUDA 版本 |
| Conda 发行版 | **Miniforge3**，`/home/lyh/miniforge3` | ✅ |
| Conda 环境 | `rl-flight` | ✅ |
| Python | **3.10.22**，解释器 `/home/lyh/miniforge3/envs/rl-flight/bin/python` | ✅ |
| PyTorch | **`2.12.1+cu130`** | ✅ GPU 矩阵/张量计算通过 |
| PyTorch CUDA runtime | **`13.0`**（`torch.version.cuda`） | ✅ |
| TorchRL | **`0.13.3`** | ✅ |
| TensorDict | **`0.13.0`** | ✅ |
| PyYAML | **`6.0.2`** | ✅ |
| `rl-flight-train` | **`0.1.0`**，可编辑安装 | ✅ |
| NumPy / SciPy | `Identification` 的依赖；当前未记录精确版本 | ⬜ 待核实 |
| Git / GitHub CLI | `git` 和 `gh` 均可使用，`gh` 已登录 | ✅；具体版本待补记 |
| 代码编辑器 | VS Code（Windows/WSL2 工作流） | 🟡 有使用计划；本项目 `code .` 实际打开结果待确认 |
| 代理软件 | Windows 上的 Clash Party，监听本地端口 **7890** | ✅ 既往使用记录 |
| WSL 网络 | `mirrored` 模式；曾用 `wslinfo --networking-mode` 核对 | ✅ 历史快照 |
| 代理环境变量 | `.bashrc` 中设置 `http_proxy/https_proxy/all_proxy` 及大写形式为 `http://127.0.0.1:7890`，设置 `no_proxy` | ✅ 曾核验；需在 Clash 运行时使用 |

### 路径和存储策略

```text
Windows D:\
└── WSL\
    ├── Downloads\
    │   └── ubuntu-24.04.5-wsl-amd64.wsl       # 当时下载的 WSL 镜像
    └── Ubuntu-24.04\                       # Ubuntu WSL 安装位置 / 虚拟磁盘

WSL Ubuntu（这些内容位于 D 盘虚拟磁盘内）
/home/lyh/
├── miniforge3/
│   └── envs/rl-flight/
└── projects/
    └── RL_Flight/                          # 个人 GitHub Fork 的工作副本
        ├── Train/
        ├── Identification/
        ├── Controller/
        ├── Deploy/
        ├── SimEnv/                          # 目前仅有配置，非完整 simenv Python 包
        └── ...

# 若将来获得独立 SimEnv 源码，预期放置位置（当前不存在）：
/home/lyh/projects/TeleaiTrans_Simenv/
```

**存储约束**：新项目、模型、数据集尽量存放 `/home/lyh/projects/`（Ubuntu 自己的 ext4 文件系统，底层位于 D 盘），不要把 Linux 高频训练工程直接放在 `/mnt/c/`。Windows/WSL 运行时有少量系统文件占用 C 盘是正常现象。

---

## 2. 到目前为止的操作时间线（按先后顺序）

> 日期依据历史对话整理。安装最初几步可能跨越 2026-10-23～10-26；未逐条保留的精确时间不作猜测。每一阶段都记录“实际操作 → 验证 → 结果”。

### 阶段 A｜2026-10-23～10-26：从 Ubuntu 双系统切换到 WSL2

**做了什么**

1. 放弃原 Ubuntu 双系统的开发方式，选择 **Windows 11 + WSL2 + Ubuntu 24.04**。
2. 配置 Windows WSL 组件，确认 WSL2 可用。
3. 在 D 盘规划 WSL 存储目录 `D:\WSL`，而不是默认将大体积 Ubuntu 数据放在 C 盘。
4. 最终使用本地官方 `.wsl` 映像安装 Ubuntu 24.04.5：

   ```powershell
   wsl --install --from-file "D:\WSL\Downloads\ubuntu-24.04.5-wsl-amd64.wsl" --location "D:\WSL\Ubuntu-24.04"
   ```

5. 初始化 Ubuntu 用户 `lyh`，完成 `sudo` / `apt update` 基础验证。

**当时遇到的问题**

- `wsl --list --online` 曾超时，后来恢复可查询。
- 直接在线安装 `wsl --install -d Ubuntu-24.04 --location ...` 曾出现 `WININET_E_CANNOT_CONNECT`。
- **处理**：改为先获取正确格式的 `.wsl` 镜像，再通过 `--from-file` 离线安装。
- 首次 NAT 网络模式下出现 localhost 代理无法透传的提示；属于网络/代理配置问题，而非 Ubuntu 安装本身失败。

**验证结果**：Ubuntu 正常启动；Linux 用户和 `apt update` 可用。✅

### 阶段 B｜2026-10-24～10-26：WSL 网络、代理与 GPU

**做了什么**

1. 将 WSL2 网络模式配置为 `mirrored`。
2. 使用 Windows Clash Party 的本地 HTTP 代理 `127.0.0.1:7890`，并设置 Ubuntu shell 的代理环境变量（含 `http_proxy`、`https_proxy`、`all_proxy`）。
3. 使用 `curl -I -x http://127.0.0.1:7890 https://github.com` 及相关检查验证代理连接，曾返回 `HTTP/1.1 200 Connection established` 与 HTTP 200。
4. 在 Ubuntu 中执行 `nvidia-smi`，看到 RTX 4060 Laptop GPU。

**当时遇到的问题**

- NAT 网络模式不能自动将 Windows 的 `localhost` 代理直接映射到 WSL 内的同名地址。
- GitHub 等网络访问有时慢或失败。
- **处理**：使用 `mirrored` 网络模式，确认 WSL 内可以经 Windows Clash 的 `127.0.0.1:7890` 建立 HTTPS CONNECT 隧道。

**验证结果**：网络与 GPU 设备可访问。✅  
**注意**：代理软件没开、切换模式、重启网络后若失效，应重新检查，不能根据历史 200 响应断言永远正常。

### 阶段 C｜2026-10-25～10-26：Conda + PyTorch CUDA

**做了什么**

1. 安装 Miniforge3 到 `/home/lyh/miniforge3`。
2. 创建并激活 `rl-flight` Conda 环境，Python **3.10.22**。
3. 安装 **PyTorch 2.12.1+cu130**，验证 `torch.cuda.is_available() == True`。
4. 在 RTX 4060 上实际执行 PyTorch GPU 张量/矩阵计算，测试通过。

**注意**

- 项目原作者文档部分位置使用 **V100 + cu126**，这是**原作者的运行环境**，不是本机。我们的 RTX 4060 **已实测 cu130 可用**，无需为了照抄 V100 文档而重装 Torch。
- `nvidia-smi` 显示的 CUDA 版本与 `torch.version.cuda` 不是同一个概念。
- WSL 运行 PyTorch 官方 CUDA wheel 通常不需在 Ubuntu 内额外安装一整套 Linux NVIDIA 显卡驱动。

**验证结果**：GPU PyTorch 计算通过。✅

### 阶段 D｜2026-10-26：TorchRL / TensorDict / PyYAML

**执行过的安装命令**

```bash
conda activate rl-flight
python -m pip install \
  "torchrl==0.13.3" \
  "tensordict==0.13.0" \
  "PyYAML==6.0.2"
```

**实测输出（节选）**

```text
PyTorch: 2.12.1+cu130
TorchRL: 0.13.3
TensorDict: 0.13.0
PyYAML: 6.0.2
CUDA: 13.0
GPU available: True
TensorDict batch: torch.Size([8])
Observation shape: torch.Size([8, 21])
Tensor device: cuda:0
PPO / SAC imports: OK
TorchRL + TensorDict test: OK
```

**验证结果**：版本、CUDA 张量、相关导入全部通过。✅  
**重要区别**：这只说明这些包能协同运行，**并不代表完整 PPO/SAC 飞行训练已跑通**。目前公司要求也不需要做这条训练路线。

### 阶段 E｜2026-10-26：Fork 自己的 GitHub 仓库并配置远程

**目的**：自己的修改只向个人仓库推送，同时保留从原作者仓库获取更新的能力。

1. 在 GitHub 将 `Ryan-5853/RL_Flight` Fork 到 `Li-Yuhang7683/RL_Flight`。
2. 在 WSL2 的 `~/projects/` 克隆**自己的 Fork**：

   ```bash
   mkdir -p ~/projects
   cd ~/projects
   git clone https://github.com/Li-Yuhang7683/RL_Flight.git
   cd RL_Flight
   git remote add upstream https://github.com/Ryan-5853/RL_Flight.git
   ```

3. 已确认的远程配置：

   ```text
   origin    https://github.com/Li-Yuhang7683/RL_Flight.git (fetch)
   origin    https://github.com/Li-Yuhang7683/RL_Flight.git (push)
   upstream  https://github.com/Ryan-5853/RL_Flight.git (fetch)
   upstream  https://github.com/Ryan-5853/RL_Flight.git (push)
   ```

4. 安装 GitHub CLI `gh`，通过 `gh auth login --hostname github.com --git-protocol https --web` 登录。
5. 执行 `gh auth setup-git`；`gh auth status` 显示**已登录 Li-Yuhang7683，协议为 HTTPS**。
6. 设置 Git 的全局提交身份：

   ```bash
   git config --global user.name "Li-Yuhang7683"
   git config --global user.email "322373396+Li-Yuhang7683@users.noreply.github.com"
   ```

**遇到的问题及解决**

- `gh auth login --web` 尝试调用 Linux 浏览器失败：`xdg-open,x-www-browser,www-browser,wslview ... not found`。
- **处理**：在 **Windows 浏览器**手动访问 `https://github.com/login/device` 完成设备授权；CLI 随后显示登录成功。
- 设备一次性验证码、GitHub Token 都不应存入日志或提交到仓库。此处**未记录**验证码或令牌。

**验证结果**：GitHub 登录、远程 URL、提交身份已确认。✅  
**仍待核验**：`git push --dry-run origin main` 是否成功，以及首次真正的功能分支推送；此前未收到这两项的输出。⬜

### 阶段 F｜2026-10-26：安装 RL_Flight Train 入口并测试

**执行命令**

```bash
conda activate rl-flight
cd ~/projects/RL_Flight
python -m pip install --no-deps -e ./Train
python -m pip check
flight-train --help
```

**实测输出（节选）**

```text
Successfully installed rl-flight-train-0.1.0
No broken requirements found.
usage: flight-train [-h] {run,evaluate} ...
```

**测试记录**

```bash
cd ~/projects/RL_Flight/Train
python -m unittest discover -s tests -p 'test_progress.py' -v
```

```text
test_continuation_rate_uses_only_steps_added_in_this_session ... ok
test_non_tty_progress_reports_stages_metrics_and_finish ... ok
test_rolling_rate_uses_at_most_the_latest_eight_rollouts ... ok
Ran 3 tests in 0.006s
OK
```

**验证结果**：Train 包、命令行入口、进度管理单测已通过。✅  
**范围限制**：只验证训练框架基础功能，未启动真实飞行仿真、未完成端到端训练。

### 阶段 G｜2026-10-26：检查 SimEnv 依赖

**执行过的检查**

```bash
python - <<'PY'
import importlib.util
spec = importlib.util.find_spec("simenv")
print("SimEnv module:", spec)
PY

ls -d ~/projects/TeleaiTrans_Simenv \
      ~/projects/RL_Flight/TeleaiTrans_Simenv \
      2>/dev/null || true
```

**实际结果**

```text
SimEnv module: None
SimEnv 尚未安装
# 两个预期源码目录均不存在
```

**分析**：

- 主仓库 `SimEnv/configs/` 包含 YAML 参数，但**不等于**完整的 `simenv` Python 动力学库。
- `Train` 完整飞行动力学训练、`Identification` 从零生成仿真数据、非线性闭环审计，都依赖独立的 SimEnv 源码/接口。
- 辨识链路中的 `control_evaluation._nominal_actuator_model()` 以及 `offline_log_inference` 的**增益合成**仍可能调用 `simenv.config.load_and_materialize`；因此“只跑网络预测”和“完全运行原版日志→LQI 增益程序”不能混为一谈。
- 公开渠道未核实到可直接克隆的 `TeleaiTrans_Simenv` 源；需要项目提供方确认授权来源。

**目前决策**：⛔ 模块缺失，但**不影响先读代码**；目前不为此安装错误的 PyPI 同名包，不搭建仿真。

### 阶段 H｜2026-10-26～11-02：公司任务聚焦、进入源码学习

**领导给出的明确方向（概括）**：

- 仅研究“**神经网络辨识 LQR 控制器**”相关实验；后续工作以这一套为基础。
- **不做端到端 NN 飞行控制**，不把 PPO/SAC 当作当前研究重点。
- 先把代码与算法弄明白；之后到公司在自己的电脑上跑一次训练。
- 后续公司会提供一架飞机，要求从数据获取到控制器参数生成、验证，完成重新辨识的流程。

**2026-11-02 当前决定**：先阅读代码，不处理 SimEnv，不跑新的实验。✅

**特别厘清**：原项目最新相关方案是**离线神经网络动力学辨识 + 四输出 LQI**。日常口语称“LQR”，代码中常用带积分状态的 LQI；**不是**网络直接输出舵机 PWM 的端到端控制。对于新飞机，先使用已训练的通用辨识器做离线推断与增益合成；如领导明确要求“从头训练神经网络”，则另需通用训练数据和标签、对应仿真/标注条件。

---

## 3. 与公司任务匹配的算法路线（只保留这一条）

```text
多架训练用飞机的闭环飞行数据（仿真/有标签）
    │  14 通道 / 500 Hz / 每次 2 s / 按“机体”划分 train、val、test
    ▼
数据整理、有效性掩码、标准化
    ▼
监督学习的辨识网络（MLP、TCN、BiGRU）
    │  不是直接学电机输出
    ▼
预测三轴“舵机命令 → 角加速度”的短时阶跃响应
    │  10 / 20 / 40 / 80 / 160 ms
    ▼
将响应拟合为固定稳定滤波基底，建立离散状态空间 A_d、B_d
    ▼
DARE / LQI 增益综合：K=(R+BᵀPB)⁻¹BᵀPA
    │  正式四输出结构：K 形状 [4,19]
    ▼
固定标称 vs 辨识增益的闭环评估、质量门控、HIL / shadow 验证
    ▼
在满足工程安全条件后再考虑真实飞行使用（目前仅研究，未获飞行授权）
```

**正式四输出结构**：上桨由飞手/外部定高控制，下桨和三个舵机由 LQI 输出，输入维数 4；包含积分状态的等效模型共 19 状态。训练的 `--target-representation step_response --adaptive-mode-indices 0,1,2` 对应 5 个时间点 × 3 个响应轴 × 3 个模态 = **45 个监督目标**。注意程序 CLI 默认的 `adaptive-mode-indices` 可能是 `1,2`，**复现实验需显式采用正式报告的 `0,1,2`**。

**真实日志的 14 个输入通道**：

| 字段组 | 数量 | 字段 |
|---|---:|---|
| 倾斜姿态 | 4 | `attitude_q_tilt.w/x/y/z`（去 yaw） |
| 角速度 | 3 | `angular_velocity_b.x/y/z` |
| 电机转速 | 2 | `motor_speed.upper/lower` |
| 电机控制指令 | 2 | `command.motor_upper/lower` |
| 舵机控制指令 | 3 | `command.servo_1/2/3` |
| **合计** | **14** | 每次 500 Hz × 2 s = `1000×14` 的原始轨迹 |

**模型输入细节**：训练前可按架构下采样（MLP 通常 100 Hz；TCN/BiGRU 500 Hz）。`prepare_histories()` 额外拼接一个时间有效性标记，因此模型读取的最后一个特征维度通常是 **15**；还支持每架飞机多次飞行的 `trial_mask`。

**不要混淆**：原始仿真生成的“监督标签”依靠带参数的模拟机体构造；新飞机的日志**没有这些真值标签**，要靠训练好的网络进行推断，而不是把少量真机日志直接当作完整的原监督数据集从零训练。

### 项目文档里的已有基线（不是本机实测成绩）

- 正式实验文档：`Identification/OFFLINE_4OUT_COMPOSITE_REPORT_zh.md`（2026-08-05）。
- 复现报告：`Identification/LQR_4OUT_REPRODUCTION_20260806_zh.md`（2026-08-06）。
- v4 原始数据配置：8,192 个不同机体 × 每机 8 次闭环试飞；报告推荐 **TCN-roll + BiGRU-pitch + MLP-yaw** 的混合辨识器和 **80% 候选增益混合**。
- 原始报告中预测增益 80% 混合相对固定复合标称 LQI 的收敛率提升约 **+10.60 / +9.75 个百分点**（两个评估种子）。这是**项目已有文档结果**，不能写成“本人已复现”。
- 复现报告中，在 RTX 4060 8GB 上使用不同 `micro-batch-size`（MLP 128、TCN 32、BiGRU 4）进行分块反向传播；它是**报告作者的复现记录**，仍不是本机已执行。
- 文档明确将候选标记为 `research_only` / `gain_accepted_for_flight=false`；没有完成 HIL 和实机 shadow 验证前，不能把仿真局部稳定性当作飞行安全证明。

---

## 4. 源码阅读路线与进度（从调用链而不是目录名开始）

> ✅ 表示已完成真正的阅读、整理与理解；目前均**未标为已掌握**。先看调用链，再进入每个算法细节。

| 顺序 | 入口 / 文件 | 需要搞清楚的问题 | 阅读进度 |
|---|---|---|---|
| 0 | `Identification/LQR_4OUT_REPRODUCTION_20260806_zh.md` | 最新正式实验是什么，历史实验与四输出方案有何区别 | 🟡 已定位文档，待系统精读 |
| 1 | `Identification/configs/lqr_sim2real_micro_offline_logs_v4.yaml` | 一次实验的机体数、初始条件、采样率、输出目录、控制器配置 | 🟡 已梳理概览，待逐字段阅读 |
| 2 | `Identification/src/flight_identification/repeated_trial_experiment.py` | 如何生成不同机体参数、闭环日志和按机体分割的数据 | ⬜ |
| 3 | `Identification/src/flight_identification/experiment.py` | 14 通道 `FEATURE_NAMES`、真实标签、数据结构 | ⬜ |
| 4 | `Identification/src/flight_identification/repeated_trial_training.py` | 数据读取、标准化、mask、组内多次试飞融合 | ⬜ |
| 5 | `Identification/src/flight_identification/sim2real_composite_training.py` | **正式监督训练入口**：训练/验证/测试、损失函数、反传、checkpoint | 🟡 已定位 `train()`（约第 195 行），待逐段精读 |
| 6 | `Identification/src/flight_identification/offline_models.py` | MLP/TCN/BiGRU 编码结构、`OfflineTemporalSetIdentifier`、张量维度 | ⬜ |
| 7 | `Identification/src/flight_identification/sim2real_composite.py` | 阶跃响应 → 滤波器参数 → 离散模型 `A_d,B_d`（19 状态） | ⬜ |
| 8 | `Identification/src/flight_identification/control_evaluation.py` | `_nominal_actuator_model()`、`_lqr_gain()`、DARE 与局部稳定性检查 | ⬜ |
| 9 | `Identification/src/flight_identification/offline_log_inference.py` | 给一架新飞机的飞行日志，如何进行离线辨识及合成增益 | ⬜ |
| 10 | `Identification/scripts/evaluate_real_log_hybrid.py` | MLP+TCN+BiGRU 分轴组合、top-8 选择、分析增益、输出 JSON | ⬜ |
| 11 | `Controller/src/flight_controller/classical.py`、`plant.py` | 控制器如何使用模型与增益，如何得到配平点/控制效能 | ⬜ |

**建议本次阅读从 `sim2real_composite_training.py::train()` 起步**，先记录各阶段输入输出张量形状；接着再回溯 `load_repeated_split()`、`true_servo_mode_step_response()` 等调用。不要只记函数名；每读完一段应写明：**数据从哪里来 → 做了什么 → 返回什么 → 在后面哪里被用到**。

**文档链接（个人 Fork）**：

- [最新四输出实验复现报告](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/LQR_4OUT_REPRODUCTION_20260806_zh.md)
- [四输出离线辨识正式报告](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/OFFLINE_4OUT_COMPOSITE_REPORT_zh.md)
- [离线真实飞行日志规范](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/OFFLINE_LOG_IDENTIFICATION_zh.md)
- [辨识训练主程序](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/src/flight_identification/sim2real_composite_training.py)
- [模型结构](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/src/flight_identification/offline_models.py)
- [离散模型构建](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/src/flight_identification/sim2real_composite.py)
- [LQR 增益求解](https://github.com/Li-Yuhang7683/RL_Flight/blob/main/Identification/src/flight_identification/control_evaluation.py)

---

## 5. 已完成、缺失与未验证事项（防止以后重复操作）

### ✅ 已完成、已有实际输出支撑

- [x] WSL2 + Ubuntu 安装在 D 盘。
- [x] Miniforge3、`rl-flight`、Python 3.10。
- [x] WSL2 GPU 识别与 PyTorch CUDA 计算。
- [x] TorchRL、TensorDict、PyYAML 的指定版本及基础张量功能。
- [x] 个人 RL_Flight Fork、本地 Clone、`origin/upstream` 配置。
- [x] GitHub CLI HTTPS 登录、Git 提交用户名和 noreply 邮箱。
- [x] `rl-flight-train==0.1.0` 可编辑安装。
- [x] `python -m pip check` 无依赖冲突、`flight-train --help` 可用。
- [x] 训练进度测试 `test_progress.py` 3/3 通过。
- [x] 查明 `simenv` 模块不在当前环境；将其暂时搁置。
- [x] 明确公司研究目标：NN 系统辨识 + LQR/LQI，**不需要端到端 RL**。

### ⬜ 尚未完成 / 待实际验证

- [ ] 本地 `git push --dry-run origin main` 成功输出；首次推送自己创建的功能分支。
- [ ] 确认 Git 当前分支、工作区状态、项目 HEAD commit SHA（保留可追溯的源码版本）。
- [ ] 记录 `git --version`、`gh --version`、`numpy` / `scipy` 版本、当前 WSL 和驱动版本。
- [ ] 确认 VS Code 能直接在 WSL 项目目录打开并正常索引。
- [ ] 安装并检查 `Identification/` 和 `Controller/` 包（仅当开始执行其相关代码时）；**此前只提出命令，没有成功执行记录**。
- [ ] 运行 Identification 不依赖 SimEnv 的单元测试（如 `test_training.py`），留存输出。
- [ ] 研究原始数据文件结构、有效性掩码、样本标准化与标签生成。
- [ ] 逐段理解三网络模型结构与训练损失。
- [ ] 搞清 19 状态、4 输入的 LQI `A_d/B_d/Q/R/K` 与配平点。
- [ ] 阅读现成检查点与已有真实日志的离线辨识链路；**执行完整增益合成可能仍依赖 SimEnv 的标称参数加载**。
- [ ] 与领导进一步对齐“跑一遍训练”是指**复现通用监督网络训练**，还是**对一架新飞机使用已训练网络离线重新辨识**；两者的数据要求与工作量不同。

### ⛔ 暂缓处理的阻塞 / 后置项目

- [ ] 获取经授权的 `TeleaiTrans_Simenv`，需要时再安装。
- [ ] 从零生成 v4 仿真数据与复现端到端“辨识训练 + 闭环评估”。
- [ ] SimEnv 非线性参数随机化、HIL、真正新飞机的安全测试。
- [ ] 取得公司新飞机的日志、飞控固件通道定义、传感器与执行器标定信息。

**无需完成**（除非公司任务变更）：PPO/SAC 正式训练、端到端策略网络部署、额外强化学习仿真框架搭建。

---

## 6. 典型问题 / 经验教训索引（持续追加）

| ID | 问题 | 原因或判断 | 解决办法 / 当前状态 |
|---|---|---|---|
| P01 | `wsl --list --online` 超时、联网安装失败 | 网络连接 / Windows 下载通道失败 | ✅ 通过下载 Ubuntu `.wsl` 文件并 `--from-file` 安装 |
| P02 | NAT 模式 `localhost` 代理提示 | NAT 下 Windows localhost 代理不自动等价于 WSL localhost | ✅ 改用 `mirrored`，并核验 `127.0.0.1:7890` HTTP CONNECT |
| P03 | GitHub 下载慢、请求受代理影响 | WSL 与 Windows Clash 代理/环境变量有关 | ✅ 配置 `.bashrc` 代理并通过 `curl` 验证；代理需保持运行 |
| P04 | 看到 CUDA 13.4 与 `cu130` 不一致 | `nvidia-smi` 驱动支持版本 ≠ PyTorch Wheel CUDA runtime | ✅ PyTorch CUDA `13.0` 且 GPU 计算实测通过，无须因此重装 |
| P05 | `gh auth login` 无法弹出浏览器 | WSL 内没有 `xdg-open` / `wslview` 等可用打开器 | ✅ 使用 Windows 浏览器访问 GitHub 设备授权页手动授权 |
| P06 | `importlib.util.find_spec('simenv') == None` | 独立 SimEnv 动力学包尚未取得，主库 `SimEnv/` 只有配置 | ⛔ 暂缓；代码阅读不受阻，数据生成/完整增益合成存在依赖 |
| P07 | 原仓库文档与当前任务不完全一致 | README 包含旧的 V100、PPO、SAC、在线/端到端探索历史 | ✅ 以四输出**离线辨识**实验文档和对应代码为学习主线 |
| P08 | 复现报告 TCN 参数规模与历史原报告不完全一致 | 原报告的 TCN 部分通道配置未完整保存；复现使用代码默认结构 | 🟡 未来重跑时固定 commit、配置、种子、模型参数量，注明差异，不能硬说逐位复现 |
| P09 | `git push` 权限尚缺实际测试 | 已登录 GitHub 不等于已有成功推送记录 | ⬜ 待读取 `git push --dry-run` 或新功能分支真实推送输出 |

---

## 7. 恢复工作 / 快速环境重检（只在需要时运行）

**Windows PowerShell：**

```powershell
wsl -d Ubuntu-24.04
```

**WSL Ubuntu：**

```bash
conda activate rl-flight
cd ~/projects/RL_Flight

# Python 和主要依赖版本
python - <<'PY'
import sys, torch, torchrl, tensordict, yaml
print('Python:', sys.version.split()[0])
print('PyTorch:', torch.__version__, 'CUDA:', torch.version.cuda)
print('GPU:', torch.cuda.is_available())
print('TorchRL:', torchrl.__version__)
print('TensorDict:', tensordict.__version__)
print('PyYAML:', yaml.__version__)
try:
    import numpy, scipy
    print('NumPy:', numpy.__version__, 'SciPy:', scipy.__version__)
except ImportError as exc:
    print('NumPy/SciPy:', exc)
PY

python -m pip check

# Git 与关联代码版本
git status --short --branch
git log -1 --format='%h %cs %s'
git remote -v

git --version
gh --version
gh auth status

# WSL 代理状态（不要把令牌贴到日志中）
wslinfo --networking-mode
```

> `gh auth status` 在部分版本会显示被遮盖的 Token；**不要手动复制未遮盖的认证信息**。日志中只记录账号、协议、成功/失败状态。

**关于 Git 同步**：不要不看工作区就直接 `git reset --hard`、`git pull` 或 `git push --force`。提交代码前检查 `.gitignore`、`git status`；训练数据、模型、日志不要无意上传到公共 GitHub。

---

## 8. 资料、产物和复现证据管理（原安装笔记容易遗漏）

### 推荐以后每次实验额外记录

- **Git 状态**：本地 commit SHA、分支、是否有未提交修改、`origin/upstream` 对应版本。
- **实验命令与配置**：完整命令、配置文件路径及 hash、随机种子、输入数据版本。
- **硬件环境**：PyTorch / CUDA / NVIDIA 驱动版本、GPU 型号、显存、`pip freeze` 摘要。
- **数据来源与质量**：sim2real 仿真数据或真机 ULog、飞行架次、采样率、无效片段、分组 train/val/test 方法。
- **训练过程**：模型结构、参数量、batch / micro-batch、epoch、学习率、损失、最佳模型路径。
- **结果与真实性**：训练/验证/测试误差、yaw 轴质量、局部极点、稳定率、对照组和置信区间；注明是**论文/仓库文档结果**还是**自己实际运行结果**。
- **安全边界**：离线分析 / HIL / shadow / 实机控制权限，不把“有增益矩阵”当作“可以安全起飞”。
- **安全和保密**：日志不存 GitHub Token、验证码、密码；公司真实飞机参数、原始 ULog 或内部资料未经许可不要放公共 Fork。

### 本仓库已知参考产物（代码仓库自带，尚非本人训练）

```text
Identification/runs/repro_20260806/
├── sim2real_offline_logs_step_response_mlp_v7/identifier.pt
├── sim2real_offline_logs_step_response_tcn_500hz_v7/identifier.pt
├── sim2real_offline_logs_step_response_bigru_500hz_v7b/identifier.pt
└── sim2real_offline_hybrid_v7/
    ├── manifest.json
    ├── offline_log_quality_calibration_v1.json
    └── control_audit_*.json

Identification/datasets/real_logs_26_8_12_v1/
├── bundle.npz                        # 仓库已有的标准化真实日志样本
├── README.md
└── ...
```

**提醒**：`manifest.json` 中可能保存了原作者机器上的绝对路径（例如 `/home/ryan_wsl/...`）；不能直接当作本机可用路径。部分训练目录的全部原始 v4 数据集可能不在 Git 仓库里，不能因看见报告就假定已有全量训练数据。

---

## 9. 面向新飞机重新辨识：未来工作清单（未开始）

1. **确认新飞机的控制接口**：上下桨/三舵机通道、旋转方向、归一化 PWM/实际执行器响应、机体系 FRD/NED 和四元数约定。
2. **确认日志可用性**：PX4 ULog 是否有姿态、角速度、两台电机转速和 5 路最终执行器指令；时间戳是否同步、采样率是否满足要求。
3. **日志清洗**：去重、缺测处理、500 Hz 对齐、去 yaw 姿态、生成标准 NPZ（`features: [flight,1000,14]`）、有效性掩码与来源元数据。
4. **离线辨识**：读取经验证的 MLP/TCN/BiGRU checkpoint，选取信息量充足的多次闭环日志，预测新的等效动态。
5. **增益综合**：构造 `A_d,B_d`，指定 Q/R，求解 DARE 与候选 LQI 增益；检查有限值、闭环局部极点与增益混合裕度。
6. **安全评估**：对齐真实执行器/姿态估计链路、非线性仿真或其他可信验证、HIL、shadow；未经验证**不直接让新控制器接管飞行**。
7. **是否重训通用网络**：如果公司说的“重新辨识”包括从零训练监督模型，必须另行确认有足够的机体多样性、参数标签或可信仿真数据，不能从单架飞机有限闭环日志凭空得到原训练集的真值监督标签。

---

## 10. 后续日志模板（每次操作追加一则）

> 新增日志按时间从新到旧放在本节顶部；每次同时更新第 0 节的摘要与第 5 节的待办状态。以下模板可直接复制。

````markdown
### YYYY-MM-DD｜标题（例如：阅读 offline_models.py）

- **目标**：
- **当前版本**：Git 分支 / commit、Python/PyTorch 版本（有变化才写）
- **涉及文件**：`Identification/...`
- **执行的命令 / 代码位置**：
  ```bash
  # 完整可复现命令（如果有）
  ```
- **结果与证据**：终端输出、关键张量维度、截图/报告文件路径
- **遇到的问题及定位**：
- **解决办法 / 尚存疑问**：
- **状态**：✅ 已验证 / 🟡 部分完成 / ⬜ 待做 / ⛔ 阻塞
- **下一步**：
````

### 2026-11-02｜确认学习范围，建立长期日志

- **目标**：结束“安装环境”阶段的反复排查，集中阅读神经网络辨识 LQR/LQI 源码。
- **已知状态**：Train 框架入口和 3 项进度测试已通过；SimEnv 缺失但当前无需处理；GitHub Fork 与登录配置已完成。
- **当前决策**：不运行 PPO、SAC，不补建仿真环境，先从 `sim2real_composite_training.py::train()` 理解数据、标签、训练循环，进而分析 `offline_models.py`、`sim2real_composite.py` 和 `_lqr_gain()`。
- **下一步**：阅读训练主程序和数据管线，并记录每个函数的输入输出形状。
- **状态**：✅ 明确范围；⬜ 正式逐段代码精读尚未开始。

---

## 11. 保存与版本管理建议

- **推荐文件名**：`RL_Flight_环境搭建与研究开发日志.md`。
- **优先存储**：WSL 的 `/home/lyh/notes/` 或 D 盘其他个人目录（均属 D 盘），以便长期更新。
- 如果想与项目源码一起跟踪，可以放入 `~/projects/RL_Flight/docs/`，但要先确认**不含公司内部信息**：你的 Fork 是公共仓库，`git push` 后所有人可见。
- 每次更换系统、升级关键包、修改模型配置、首次成功复现实验时，补充一则日志，不覆盖原有故障和测试证据。

**日志核心规则**：不要将“已计划”“文档里的结果”“别人已经复现”“本机确实完成”混为一谈。
