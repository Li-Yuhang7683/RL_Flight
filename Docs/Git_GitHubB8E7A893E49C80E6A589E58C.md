# Git 与 GitHub 常用操作速查手册

> 适用环境：Windows 11 + WSL2 Ubuntu 24.04，在 VS Code 的 **WSL Ubuntu 终端**操作。
> 
> 项目示例：`~/projects/RL_Flight`，自己的 GitHub Fork 为 `Li-Yuhang7683/RL_Flight`，原作者仓库为 `Ryan-5853/RL_Flight`。
> 
> 最后整理：2026-11-03。本文命令均在仓库根目录操作；除非特别标注，不要在 Windows PowerShell 里混用 WSL 路径。

## 0. 先分清 Git、GitHub、origin 和 upstream

- **Git**：管理本地代码版本、分支和提交记录的软件。
- **GitHub**：托管 Git 仓库的网站；`push` 才会把提交上传到远端。
- **工作区（working tree）**：你在 VS Code 中正在编辑的文件。
- **暂存区（staging area）**：用 `git add` 选中、等待提交的改动。
- **本地仓库（local repository）**：执行 `git commit` 后保存的历史记录。
- **`origin`**：自己 Fork 的仓库，日常向这里提交代码。
- **`upstream`**：原作者仓库，通常只拉取更新，不向这里推送。
- **`main`**：项目默认分支；功能开发最好单独创建分支。

在本项目中：

```text
VS Code 编辑的文件
      |
      v
 git add <file>       选入暂存区
      |
      v
 git commit           生成本地提交
      |
      v
 git push origin ...  上传到自己的 GitHub Fork

原作者 Ryan-5853/RL_Flight (upstream)
      |
      | git fetch upstream + git merge upstream/main
      v
本地 RL_Flight 仓库
      |
      | git push origin main
      v
自己的 Li-Yuhang7683/RL_Flight (origin)
```

> **注意**：GitHub 登录账号、`git config user.name` 和远程仓库地址是不同概念。`git config` 决定提交作者显示名，`gh auth status` 检查认证账号，`git remote -v` 决定远程仓库的地址。实际 `push` 的目标还受具体命令、当前分支和 Git 配置影响。

---

## 1. 每次进入项目先检查

```bash
cd ~/projects/RL_Flight
pwd                          # 当前目录
git status -sb               # 当前分支、工作区状态，简洁输出
git branch -vv               # 当前分支及其跟踪的远程分支
git remote -v                # origin/upstream 的 fetch、push 地址
gh auth status               # GitHub CLI 的认证情况
```

**预期的远程地址**：

```text
origin   https://github.com/Li-Yuhang7683/RL_Flight.git (fetch)
origin   https://github.com/Li-Yuhang7683/RL_Flight.git (push)
upstream https://github.com/Ryan-5853/RL_Flight.git (fetch)
upstream https://github.com/Ryan-5853/RL_Flight.git (push)
```

这并不表示 `git push` 一定会自动选 `origin`。若不确定，显式写明 `git push origin main` 或当前分支名称。

### Git 身份与认证

```bash
git config user.name              # 当前仓库生效的提交作者名
git config user.email             # 当前仓库生效的提交邮箱
git config --list                 # 所有生效配置；输出可能包含远程地址
git config --show-origin --get user.email   # 查看邮箱配置来自哪个文件

gh auth status                   # 登录状态；不要分享真实 token
gh api user --jq '.login'         # GitHub CLI 当前身份
```

需要修改作者信息时：

```bash
git config --global user.name "Li-Yuhang7683"
git config --global user.email "322373396+Li-Yuhang7683@users.noreply.github.com"
```

`--global` 影响该 WSL 用户下的全部仓库；只想修改当前仓库就去掉 `--global`。不带参数单独输入 `git config`，只会显示用法帮助。

---

## 2. 最常用：修改文件 → 提交 → 上传

假设新增或修改了 `docs/RL_Flight_环境搭建与研究开发日志.md`。

### 2.1 检查变更

```bash
cd ~/projects/RL_Flight
git status
git diff                          # 尚未暂存的改动
git diff -- docs/                 # 只查看 docs 里的改动
```

### 2.2 只暂存指定文件（推荐）

```bash
git add "docs/RL_Flight_环境搭建与研究开发日志.md"
git diff --cached --stat          # 已暂存文件列表和改动统计
git diff --cached                 # 仔细确认将要提交的完整改动
```

`git add .` 会暂存**当前目录及其子目录的所有未忽略改动**。公司数据、训练日志、凭据、意外新增文件都可能因此进入提交，谨慎使用。

### 2.3 创建本地提交

```bash
git commit -m "docs: update RL_Flight development log"
git log -1 --oneline              # 检查最近一条提交
```

> `commit` 只写入**本地 Git 历史**，不会自动上传到 GitHub。

### 2.4 安全推送到自己的仓库

先看分支：

```bash
git branch --show-current
```

若当前是 `main`：

```bash
git push --dry-run origin main   # 模拟推送，不上传
git push origin main             # 真正推送到自己的 Fork
```

若当前是 `feature/read-identification`：

```bash
git push -u origin feature/read-identification
```

也可以明确指定“把当前分支推送到远程同名分支”：

```bash
git push origin HEAD:refs/heads/$(git branch --show-current)
```

推送后检查：

```bash
git status -sb
git log -1 --oneline
```

浏览器打开：`https://github.com/Li-Yuhang7683/RL_Flight`。

---

## 3. 从 GitHub 获取更新：fetch、pull、push 的区别

| 命令 | 含义 | 会直接改变当前文件吗？ | 会上传到 GitHub 吗？ |
|---|---|---|---|
| `git fetch origin` | 下载远程分支的提交和引用 | 不会 | 不会 |
| `git pull --ff-only origin main` | 下载并仅在可以快进时更新当前分支 | 会 | 不会 |
| `git pull --rebase origin main` | 下载、更新基础提交并重放本地提交 | 会 | 不会 |
| `git push origin main` | 上传本地 `main` 提交 | 不会 | **会** |

**日常保守同步流程**（确认当前在 `main`，并且本地无未提交修改）：

```bash
cd ~/projects/RL_Flight
git status -sb
git fetch origin
git log --oneline --left-right HEAD...origin/main
```

差异输出含义：

- `<`：只存在本地的提交；
- `>`：只存在 `origin/main` 的提交；
- 无输出：双方指向相同提交或没有相互独有的提交。

如果本地没有独有提交，可以执行：

```bash
git pull --ff-only origin main
```

如果本地和远端都有新提交，先检查差异并决定合并或 rebase，**不要强推覆盖远端**。

### VS Code 的“同步更改”按钮

VS Code 弹窗“从 `origin/main` 中拉取并向其推送提交”，表示一次同步操作包含 **Pull + Push**，不等于“只上传”。首次使用建议在终端执行上述 `fetch`、检查差异、明确 `push` 的流程，遇到分叉先处理。

---

## 4. 从原作者 upstream 同步最新代码

**前提**：你需要把原作者的更新合入自己的 Fork，而且本地改动已提交或暂存，当前位于要更新的 `main` 分支。

```bash
cd ~/projects/RL_Flight
git status -sb
git switch main
git fetch upstream                 # 获取原作者最新代码
git log --oneline --left-right main...upstream/main  # 先看差异
git merge upstream/main            # 把原作者 main 合入本地 main
git push origin main               # 同步更新自己的 Fork
```

如果只允许“无分叉、无合并提交”的快进更新，可把 `git merge upstream/main` 换为：

```bash
git merge --ff-only upstream/main
```

如果提示 `Not possible to fast-forward`，说明有分叉。不要立即使用 `--force`；可以选择普通 `merge`，或单独规划如何 rebase。

如果你想**只查看原作者更新而不合并**：

```bash
git fetch upstream
git log --oneline HEAD..upstream/main
```

---

## 5. 分支管理：阅读、实验和修改尽量单开分支

```bash
git branch -a                         # 查看本地和远程分支
git branch --show-current             # 当前分支
git switch -c study/lqr-identification # 基于当前提交创建并切换新分支
git switch main                       # 回到 main
git switch study/lqr-identification    # 切到学习分支
```

在学习分支保存研究代码：

```bash
git add Identification/src/flight_identification/offline_models.py
git commit -m "study: annotate offline identifier model"
git push -u origin study/lqr-identification
```

如要把学习分支合入本地 `main`：

```bash
git switch main
git merge study/lqr-identification
git push origin main
```

只有确认不再需要旧分支时，才能删除：

```bash
git branch -d study/lqr-identification     # 删除已合并的本地分支
git push origin --delete study/lqr-identification  # 删除远程分支
```

> 删除分支是有影响的操作；不要随意使用 `git branch -D`。

---

## 6. GitHub CLI (`gh`) 常用操作

`git` 管代码版本；`gh` 是 GitHub 的命令行工具，可以管理仓库、PR 和身份。

```bash
gh auth status                      # 查看登录状态
gh repo view Li-Yuhang7683/RL_Flight # 查看指定仓库
gh repo view --web                  # 在浏览器打开当前仓库
gh repo clone Li-Yuhang7683/RL_Flight # 克隆仓库（应在未占用的父目录执行）
gh pr status                        # 当前仓库 PR 概况
gh pr list                          # 查看 PR 列表
```

通过 PR 合并学习分支（可选）：

```bash
git switch study/lqr-identification
git push -u origin study/lqr-identification
gh pr create --base main --head study/lqr-identification \
  --title "docs: document LQR identification" \
  --body "Add code-reading notes and experiments."
```

PR 创建后可在 GitHub 网页审核再合并。若你的 Fork 是公开的，PR 和分支提交也可能公开可见。

---

## 7. 查看差异、历史与文件状态

```bash
git status -sb                        # 最常用：工作区概况
git log --oneline --graph --decorate -12  # 最近 12 条提交树
git log --oneline -- docs/            # docs 文件夹的历史
git show HEAD                         # 最近提交的详细内容
git show HEAD:README.md               # 读取最新提交中的某个文件内容
git diff                               # 尚未暂存的改动
git diff --cached                      # 已暂存的改动
git diff main..upstream/main -- README.md # 对比两个分支中的指定文件
git blame Identification/src/flight_identification/offline_models.py
```

查看某文件的提交历史：

```bash
git log --oneline --follow -- "docs/RL_Flight_环境搭建与研究开发日志.md"
```

---

## 8. 常用撤销操作：从最安全的开始

### 8.1 取消暂存（不删除文件修改）

```bash
git restore --staged "docs/RL_Flight_环境搭建与研究开发日志.md"
```

例如误用了 `git add .`，可以运行 `git restore --staged .` 清空当前目录及其子目录的暂存状态，但保留工作区修改。

### 8.2 丢弃某文件尚未暂存的改动（不可轻易恢复）

```bash
git diff -- README.md   # 先确认要丢弃的内容
git restore README.md   # 将文件恢复至暂存区版本；会丢弃工作区修改
```

### 8.3 撤销最近一个本地提交但保留修改

只适用于**尚未推送**的最近提交，先检查历史：

```bash
git log -3 --oneline
git reset --soft HEAD~1
```

文件修改仍留在暂存区。不要对已分享的提交随意 `reset` 后强推。

### 8.4 撤销已经推送的某次提交（推荐 revert）

```bash
git log --oneline -5
git revert <commit-sha>  # 新建一个反向提交，不改写共享历史
git push origin main
```

> 若提交泄露了密码或 Token，`revert` 并不会从 Git 历史中彻底清除秘密：**先立即吊销/轮换凭据**，再按泄露事件流程处理。

### 8.5 暂存未提交工作，切分支或拉代码

```bash
git stash push -u -m "WIP: reading LQR source" # 暂存改动，含未跟踪文件
git stash list                         # 查看暂存列表
git stash pop                          # 应用最近的 stash 并在成功时删除它
```

`stash pop` 可能产生冲突；未跟踪文件可能包含敏感内容，暂存不等于加密备份。

---

## 9. 合并冲突如何处理

当 `pull`、`merge` 或 `rebase` 出现冲突时：

```bash
git status
```

在 VS Code 中打开冲突文件，检查 `<<<<<<<`、`=======`、`>>>>>>>` 标记，并选择、手动整合正确内容；**不要不看就“全部采用传入更改”**。

如果执行的是 `git merge`，解决后：

```bash
git add <已解决的文件>
git merge --continue
```

放弃尚未完成的 merge：

```bash
git merge --abort
```

如果执行的是 `git rebase`，解决后：

```bash
git add <已解决的文件>
git rebase --continue
```

放弃尚未完成的 rebase：

```bash
git rebase --abort
```

---

## 10. `.gitignore` 与防止误上传

本项目中有训练数据、模型、真实日志、公司材料等，**不要默认推送所有文件**。

查看文件是否被忽略：

```bash
git check-ignore -v Identification/artifacts/example.pt
```

`.gitignore` 只对**尚未被 Git 跟踪**的文件生效。某文件已被提交过，要停止追踪但保留本地文件：

```bash
git rm --cached "path/to/private-file"
```

然后添加合适的 `.gitignore` 规则、检查 diff，再提交。注意这不会清除旧提交里的文件内容。

提交前的安全检查：

```bash
git status --short
git diff --cached --name-only
git diff --cached
```

检查是否含有：公司内部资料、真实飞机原始日志、模型权重、API token、`.env`、私钥、账号密码、个人隐私、过大的数据集。

**公开仓库不等于私人备份**。内部实验记录如未获准公开，最好存放在 `~/notes/` 或权限受控的私有仓库中，而不是公开的 `RL_Flight/docs/`。

---

## 11. 常见报错速查

| 现象或提示 | 常见原因 | 建议处理 |
|---|---|---|
| `git config` 显示 usage | 没有指定参数 | 用 `git config user.name`、`git config --list` |
| `nothing to commit, working tree clean` | 没有待提交的修改 | 无需重复提交，检查 `git log -1` |
| `Everything up-to-date` | 远端已包含本地提交 | 无需再次推送 |
| `rejected (non-fast-forward)` | 远端有未包含在本地的提交 | `git fetch origin`，检查差异，合并或 rebase；不要盲目 `--force` |
| `fatal: not a git repository` | 终端不在项目仓库内 | `cd ~/projects/RL_Flight` |
| `Permission denied` / `Authentication failed` | GitHub 凭证或仓库权限问题 | `gh auth status`，检查 `gh auth setup-git` 和访问权限 |
| VS Code 提示“拉取并向其推送” | 点击了“同步更改” | 如果只想上传，使用明确的 `git push origin <branch>` |
| `Your local changes would be overwritten` | 工作区修改可能与拉取/切换冲突 | 先 `git status`，提交或 `stash` 后再继续 |
| 冲突标记出现在文件中 | Merge/Rebase 冲突 | 在 VS Code 处理冲突，`git add` 后 `--continue` |

---

## 12. 四套直接可复制的日常流程

### A. 只更新 Markdown 日志并推送

```bash
cd ~/projects/RL_Flight
git status -sb
git add "docs/RL_Flight_环境搭建与研究开发日志.md"
git diff --cached
git commit -m "docs: update development log"
git push origin main    # 仅当当前分支确实是 main
```

### B. 拉取自己 Fork 的最新 main（没有未提交工作时）

```bash
cd ~/projects/RL_Flight
git switch main
git status -sb
git pull --ff-only origin main
```

### C. 更新原作者代码到自己的 Fork

```bash
cd ~/projects/RL_Flight
git switch main
git status -sb
git fetch upstream
git log --oneline --left-right main...upstream/main
git merge upstream/main
git push origin main
```

### D. 用新分支研究神经网络辨识代码

```bash
cd ~/projects/RL_Flight
git switch main
git switch -c study/lqr-identification
# 在 VS Code 中阅读、注释、修改自己的研究代码
git status
git add "docs/代码阅读笔记.md"
git diff --cached
git commit -m "docs: add LQR identification notes"
git push -u origin study/lqr-identification
```

---

## 13. 个人操作日志模板（以后追加）

```markdown
### YYYY-MM-DD：本次 Git 操作

- 当前目录：`~/projects/RL_Flight`
- 当前分支：`main`
- `git status -sb`：
- 本次修改的文件：
- 执行的命令：
- 本地 commit SHA：
- 是否已 push 到 `origin`：
- 如同步了 `upstream`，对应的提交：
- 遇到的问题与解决办法：
- 下次需要做的事情：
```

**最后的安全口诀**：`status` 看清楚、`diff` 看内容、`add` 选文件、`commit` 存本地、`push origin` 上传自己的仓库；`fetch upstream` 查看原作者更新，确认之后才合并。
