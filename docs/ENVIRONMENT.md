# 环境与部署

## 运行环境

便携包支持 Windows x64；需要有效的图形桌面及显卡驱动。包内包含本机已有 release 可执行文件及其原生依赖，模型/音效已嵌入 exe。程序使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，其运行 DLL 一并提供。运行入口使用运行包内部路径，测试时没有把 D 盘 ROS/Pixi 目录加入 PATH。

Release ZIP 与 Git 源码分别管理：源码负责团队修改，Release 提供固定二进制版本。修改源码不会自动更新便携包，应执行 `build.ps1` 并用 `run.ps1 -Source` 启动；维护者重新生成并发布 Release 后，其他队员才能下载到新的便携版本。

## 开发依赖版本

| 依赖 | 记录 / 锁定方式 |
|---|---|
| Windows / 架构 | Windows x64；本次在原 Windows 主机验证 |
| Rust | 1.98.1，rust-toolchain.toml |
| Bevy | 0.17.2，Cargo.lock |
| Avian3D | 0.4 系列，具体解析版本见 Cargo.lock |
| r2r | Git commit `653167aadcc32b20cb13b6542460ee1926ed933c`；Cargo.lock 固定 |
| ROS 2 | Jazzy，固定 Windows 官方 ZIP：20250820 |
| Python / 原生库 | env/windows/pixi.toml + pixi.lock，Python 3.12.3 |
| MSVC | Visual Studio 2022 C++ Build Tools；本机原版本 14.44.35207 |
| Windows SDK | 本机原版本 10.0.26100.0；新入口自动读取 VsDevCmd 的 INCLUDE/LIB |
| LLVM | libclang；本机为 LLVM 23.1.1，新机需安装并验证版本兼容性 |

开发工具本身不随 Git 上传。可使用官方安装器，或在有 winget 的系统执行：

```powershell
winget install --id Git.Git -e
winget install --id GitHub.cli -e
winget install --id Rustlang.Rustup -e
winget install --id prefix-dev.pixi -e
winget install --id LLVM.LLVM -e
# Visual Studio Installer 中安装“使用 C++ 的桌面开发”、MSVC x64/x86、Windows SDK。
# winget 的 BuildTools 安装不会自动保证这些组件已勾选。
winget install --id Microsoft.VisualStudio.2022.BuildTools -e
```

安装后打开新终端，确认 `git`、`gh`、`rustup`、`cargo`、`pixi` 可用。在仓库根执行 bootstrap 和 build。Rust 的精确版本由 rustup 按工具链文件获取。建议使用较短、无空格且不含中文的克隆路径，例如 `C:\rm\RM_rune_simulator`，以减少 ROS/colcon 及 Windows 路径长度问题。

## 安装脚本做什么

1. 使用 `env/windows/ros2.json` 的固定官方 URL 和 SHA256 校验 ROS ZIP，下载到 `.downloads/`。
2. 解压到 `.deps/ros2`，使用仓库保存的 Pixi manifest/lock，执行 `pixi install --locked`。
3. 自动查找 Visual Studio 的 `VsDevCmd.bat`，加载 ROS 环境；从 `INCLUDE` 自动生成 bindgen 头文件搜索参数。
4. 将仓库消息源码复制到 `.deps/ros2_ws/src/rm_interfaces`，执行 `colcon build --packages-select rm_interfaces`。
5. `build.ps1` 加载消息 overlay 后，以 `--locked` 构建或测试 Rust 仿真器。

脚本不创建 C 盘 junction，也不修改系统 PATH、不强制结束既有仿真进程。bootstrap 可在消息源码一致时重跑；发现已有不完整目录或消息源不同则报错供人检查，避免覆盖手工修改。消息生成显式使用 Pixi Python，避免系统 Python 缺少 empy 而构建失败。

## 使用已有 ROS 环境

在 `config/local.json` 增加：

```json
{
  "ROS2_HOME": "D:/ros2/jazzy",
  "RM_WS": "D:/ros2_ws",
  "LIBCLANG_PATH": "C:/Program Files/LLVM/bin"
}
```

已有环境应包含与仓库一致的 rm_interfaces；比较 `.msg`/`.srv` 内容后再使用。新建 overlay 更便于排除旧生成物问题。使用 `scripts/build.ps1` 自动加载环境。若需在当前 PowerShell 使用 ROS Python 工具：

```powershell
. ./scripts/common.ps1
Import-SimSettings
Import-DevEnvironment
python tools/image_rate_probe.py /image_compressed 20
```

## 固定版本与复现边界

官方 ROS 下载的 SHA256：`b2decbc86bdc6103fa839aba49dd320ad7a2e00657e97e8e2778ced0106a58c8`。该下载项已向 GitHub 发布 API 核对；本次没有重新下载并在全新机器从零安装 675 MB 的 ROS ZIP。Pixi 锁文件直接来自当前可用本机环境，未经重新求解。r2r 本地 checkout 没有源码修改，只有 Cargo 的 `.cargo-ok` 标记。

运行包针对现有编译产物做了依赖闭包整理和隔离启动验证。整个开发工具链在新 Windows 上的首次安装仍须团队验收；锁定库版本不等于锁定所有驱动、系统更新、Visual Studio 与 LLVM 安装器版本。

Linux 自瞄端建议 Ubuntu 24.04 + ROS 2 Jazzy，并从本仓库编译同版本 rm_interfaces。算法本体及权重由团队自己的算法仓库管理；不从本机 `_linux_src` 快照自动复制，避免混入旧副本、模型和其他项目内容。
