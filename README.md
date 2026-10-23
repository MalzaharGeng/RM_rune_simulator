# RM Rune Simulator · 战队自瞄仿真

基于 [Blackjack200 / Daedalus](https://github.com/Blackjack200/bevy_robomaster_simulator) 的战队适配版本，主要用于 **Windows 仿真端 + ROS 2 Jazzy 自瞄端**的图像、云台控制和小陀螺闭环联调，保留能量机关场景。

参考介绍：[河北科技大学 Actor&Thinker 战队 RM2026 视觉算法仿真器开源](https://bbs.robomaster.com/article/1887395?source=4)。原作者为 Blackjack200 / Actor&Thinker；本仓库维护者为 [MalzaharGeng](https://github.com/MalzaharGeng)。项目沿用 **AGPL-3.0**，自定义消息包保留上游 Apache-2.0 许可证。

**推荐先下载便携包运行。** 仓库包含全部仿真源码、5 个模型/音效资源、消息定义和依赖锁文件；可执行程序及 ROS DLL 放在版本化的 [Release](https://github.com/MalzaharGeng/RM_rune_simulator/releases/tag/v2026.10.23)。首次获取需要联网，之后可离线运行。便携包无需安装 Rust、ROS、Python 或 Visual Studio；需要 Windows x64 和支持 Bevy/wgpu 的显卡驱动。

> 仓库名沿用 rune，但当前默认实验是 **4 m 前方、7.5 rad/s 恒速旋转的敌方步兵**。外部自瞄算法、神经网络权重、真机驱动和 PlotJuggler 本体不在本仓库中。单独运行可观察场景和真值；自动瞄准/自动开火需要自行接入自瞄节点。

## 快速运行

```powershell
git clone https://github.com/MalzaharGeng/RM_rune_simulator.git
cd RM_rune_simulator

# 首次下载 Release。私有仓库需要先 gh auth login，并获得仓库协作者权限。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/download-runtime.ps1
.\start_rune_simulator.bat
```

也可从 Release 手动下载 `windows-runtime-2026.10.23-x64.zip`：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/download-runtime.ps1 -Archive C:\Downloads\windows-runtime-2026.10.23-x64.zip
```

下载脚本校验仓库中记录的 SHA256，再解压到 `.runtime/windows/`。双击 `start_rune_simulator.bat` 即可启动。启动脚本不会杀掉其他仿真进程，也不会每次启动都重新编译。便携包来自本机 **2026-10-13 已编译版本**，本次整理未重新编译整个仿真器；修改源码后请使用下面的开发入口。

## 团队修改与构建

先安装 **Git、Rust/rustup、Pixi、Visual Studio 2022 C++ Build Tools（含 Windows SDK）及 LLVM/libclang**。具体版本、安装命令和已有环境接入方式见 [环境说明](docs/ENVIRONMENT.md)。

```powershell
# 下载并校验固定版本 ROS，按 pixi.lock 安装依赖，再编译 rm_interfaces。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/bootstrap-windows.ps1

# 完整构建、既有单元测试、运行修改后的程序。
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Test
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run.ps1 -Source
```

首次开发环境安装/构建较慢，且需要额外磁盘空间；编译缓存不会上传 Git。`Cargo.lock` 锁定 Rust 依赖，`env/windows/pixi.lock` 锁定 Windows ROS 依赖。运行包下载与本机开发构建是两个独立入口，默认启动不会误用未更新的本机编译结果。

## 配置实验

```powershell
Copy-Item config/local.example.json config/local.json
```

编辑 `config/local.json`，例如向另一台机器发送真值：

```json
{
  "GT_UDP_TARGET": "192.168.1.100:9870",
  "ROS_DOMAIN_ID": "25",
  "SPIN_MODE": "slow",
  "SPIN_SLOW_OMEGA": "7.5"
}
```

`local.json` 仅存于本机，不会被提交。优先级为：**local.json > 当前进程环境变量 > defaults.json**。跨机使用时两端设置相同 `ROS_DOMAIN_ID`，使用可达的网络地址并检查 DDS/UDP 防火墙规则。

| 配置 | 发布入口默认值 | 用途 |
|---|---|---|
| `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | `25` / `rmw_fastrtps_cpp` | ROS 发现与 FastDDS |
| `GT_UDP_TARGET` / `GT_UDP_ENABLE` | `127.0.0.1:9870` / `1` | 独立 JSON 真值；跨机时修改目标地址 |
| `SPIN_MODE` / `SPIN_SLOW_OMEGA` | `slow` / `7.5` rad/s | 当前恒速小陀螺场景 |
| `ENEMY_X` / `ENEMY_Z` | `0` / `-4` m | Bevy 世界 XZ 平面上的敌方位置 |
| `PIN_LOCAL_INFANTRY` | `1` | 我方出生点固定；在跟随视角按 WASD/QE 后解除 |
| `ARMOR_TILT_DEG` | `15`° | 装甲板内倾；设 `0` 可对照竖直安装 |
| `IMAGE_PUB` | `compressed` | `raw` / `compressed` / `both` / `none` |
| `JPEG_QUALITY` | `100` | JPEG 质量；q100 仍是 JPEG，不能视为严格无损 |
| `CAPTURE_WIDTH` / `CAPTURE_HEIGHT` | `960` / `720` | 采集尺寸，影响内参和带宽 |
| `SHOT_LOG` | 未设置 | 设置为已有目录中的文件路径后，记录逐发事件 |

五种旋转模式：`slow` 恒速；`triangle` 线性往返、无保持段；`constant` 加速到上限后保持；`regular` 上下限间交替并保持 0.35 s；`irregular` 固定种子的随机速度及保持时间。后三者可通过 `SPIN_ALPHA`、`SPIN_MIN/MAX`、`SPIN_HOLD_MIN/MAX`、`SPIN_SEED` 配置。

三角波示例：`SPIN_MODE=triangle`、`SPIN_MIN=-4`、`SPIN_MAX=4`、`SPIN_T_ACC=0.5`，对应角加速度绝对值 16 rad/s²、周期 1 s。注意直接运行 exe、绕过发布脚本时，**源码默认是 triangle / [-4,4]，不是 slow / 7.5**。

## ROS 2 与真值接口

| 方向 | 接口 | 类型 / 说明 |
|---|---|---|
| 仿真发布 | `/image_compressed` | `sensor_msgs/msg/CompressedImage`，默认开启 |
| 仿真发布 | `/image_raw` | `sensor_msgs/msg/Image`，由 IMAGE_PUB 控制 |
| 仿真发布 | `/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 仿真发布 | `/tf` | `tf2_msgs/msg/TFMessage` |
| 仿真发布 | `/gimbal_pose`、`/odom_pose`、`/camera_pose` | `geometry_msgs/msg/PoseStamped` |
| 仿真订阅 | `/rune_solver/cmd_gimbal` | `rm_interfaces/msg/GimbalCmd`，sensor_data QoS |
| 仿真作为客户端 | `/rune_detector/set_mode`、`/rune_solver/set_mode` | `rm_interfaces/srv/SetMode`，请求 mode=2；服务缺失不等于仿真启动失败 |
| 仿真发送 | UDP `9870` | JSON：`timestamp` + `gt`，独立于 ROS 图像链路 |

`GimbalCmd` 中 `yaw/pitch/yaw_diff/pitch_diff` 使用 **度**，仿真接收后转为弧度；`distance` 为米，`fire_advice` 为开火建议。位姿四元数和 UDP 真值按各自 ROS/三维坐标约定使用，UDP 角度为 **rad**。不能把命令字段的度数与真值的弧度直接相减。

UDP 真值包含云台角和位置、敌方中心、4 块装甲板中心，以及 `spin_mode/angle/omega/alpha/target/phase/event_t/t_acc`。坐标转换在 `src/ros2/plugin.rs`；跨程序比较前必须统一坐标、单位、时钟和采样时刻。真值不是 `/gt` ROS 话题，敌方装甲板真值也不通过 `/tf` 的 enemy_armor 帧提供。

**相机注意：** 当前垂直 FOV 固定 **45°**，不支持通过环境变量修改 FOV。960×720 下理想内参约为 `fx=fy=869.1169, cx=480, cy=360`，无镜头畸变。现有 `sp_vision_25` 接入曾使用 YAML 内参，并不消费 `/camera_info`；修改分辨率后，需要同步修改算法端的内参/ROI，或先实现并验证 CameraInfo 消费逻辑。

## 能测试什么

| 测试目标 | 可用场景 / 数据 | 判定重点 |
|---|---|---|
| 检测与 PnP | 装甲板图像、四角点诊断、装甲板真值 | 漏检、角点重投影、距离/姿态误差 |
| 跟踪与预测 | 五种旋转模式、可调距离/方位、固定随机种子 | 角速度估计、换板、变速与换向响应 |
| 云台闭环 | GimbalCmd → 实际姿态 → 图像反馈 | 跟踪误差、控制频率、失联行为 |
| 开火策略 | 发射/总命中/近侧命中、逐发事件 | 命中率与发射率同时评估，计入预热时间 |
| 图像链路 | raw/JPEG、分辨率与质量配置、`[CAM]` | 接收帧率、延迟、发送失败、带宽 |
| 几何一致性 | 15° 内倾和支点修复、板中心与图像 | 对称性、渲染/代理面/算法模型的差异 |
| 能量机关联调 | 继承的小/大符场景及模式服务 | 外观、旋转与激活流程；需接外部算法实测 |

**开始采集真值：** 先运行监听工具，再启动仿真。

```powershell
python tools/udp_gt_capture.py 20 9870 logs/gt.jsonl
python tools/verify_spin_gt.py logs/gt.jsonl 4 14
# 若采用上面的 [-4,4] 三角波场景，则最后两个参数改为 -4 4。
```

更多步骤、A/B 模板及已完成验证见 [测试手册](docs/TESTING.md)。历史日志原始计数及本次验证证据见 [证据说明](docs/evidence/README.md)。

`infantry_pct = 总步兵命中数 / 发射数`，`infantry_near_pct = 近侧命中数 / 发射数`；near 表示命中时板的外法线朝向炮口，**不等价于“选中了算法目标板”或真机命中率**。当前实现允许弹丸继续飞行穿过车体，多板命中和未到期弹丸会影响快照口径，应使用逐发事件去重，并在停止开火后等待超过 2 s 再统计最终结果。

历史 FINAL 日志最后一条快照为 **1729 发 / 1700 次总命中 / 1637 次近侧命中（98.32% / 94.68%）**。这是当时 Windows 仿真与 Linux 自瞄组合的记录，不能保证在其他机器、其他算法或其他实验配置下复现。历史自瞄的 pose 门和 25 s 预热门属于外部 Linux 程序，本仓库不包含这些算法实现。

## 与上游及原本机工程的区别

- 面向现有战队 ROS 2 接口适配；新增可重复的小陀螺曲线、UDP 真值和逐发日志。
- 修复装甲板内倾支点，增加图像发送仪表、MTU 1400 的 FastDDS 配置与近侧命中口径。
- 对当前场景调整发射冷却至 0.05 s、云台转速上限至 6 rad/s、弹丸寿命至 2 s。
- 本次发布增加可移植入口、固定版本依赖、完整消息源码、可校验运行包和团队文档，仿真器 `.rs` 文件保持本机现状。

完整归因、已回滚尝试和未实施建议见 [上游对比](docs/UPSTREAM_COMPARISON.md)。上游 master 已演进为更大的多目标/Talos 架构，本仓库是既有较早分支的整理版本，**不包含上游当前全部功能，也没有把上游原有功能算作本仓库新增**。

## 目录结构

```text
Cargo.toml / Cargo.lock / rust-toolchain.toml
src/                    Rust 源码
  assets/               3 个 GLB + 2 个音效，编译时嵌入
  ros2/                 图像、控制、TF、UDP 真值
  spin.rs / shotlog.rs  旋转曲线与逐发日志
rm_interfaces/          与本机使用版本对应的 ROS 消息/服务源码
config/                 团队默认配置、个人覆盖示例
env/                    运行包校验信息、ROS/Pixi 固定版本清单
scripts/                下载、环境安装、编译、运行
tools/                  真值采集/验证、模型检查、图像测速
docs/                   环境、改动对比、测试与证据
DEFAULT_FASTRTPS_PROFILES.xml
start_rune_simulator.bat
```

模型保留在 `src/assets/` 以兼容原有 `embedded_asset!` 路径。最大模型约 72 MB，可直接随普通 Git 克隆，无须 Git LFS。缓存、旧源码快照、实验日志、个人配置和编译器安装目录不提交。

## 已知边界与排障

- DLL 缺失 / `0xC0000135`：先校验并重新安装运行包，使用仓库入口；开发模式检查 ROS、Pixi 和消息 overlay 的 PATH。
- 收不到图像：检查两端 domain、消息类型、QoS、网卡和防火墙；保留根目录 FastDDS XML 及其原文件名。不要叠加另一个同名 transport 配置文件。
- 内参不一致：确认图像真实尺寸，核对算法是否确实订阅 CameraInfo，不能只看仿真是否发布。
- 当前无完整曝光/畸变/噪声/电机动力学/真实遮挡和空气阻力模型。当前弹丸碰撞球直径是 **44.5 mm**、质量 44.5 g、线性阻尼 0.05、初速 25 m/s；不能按 17 mm 真机弹丸解释命中率。弹丸 2 s 寿命不适合长滞空实验。
- 四角点诊断与 dataset 模块仍在，但当前写图/标注流程没有完成接线，**不承诺自动导出训练集**。`SHOT_LOG` 在缺少请求或重复命中时可能含 NaN，严格 JSON 解析器需专门处理。
- 本次本机验证不覆盖另一台全新 Windows 的完整开发环境安装，也不覆盖跨机 Linux 自瞄和真机实弹；应按测试手册完成团队验收。
- 旧 `verify_spin_gt.py` 的系统时间积分判据可能与 Bevy 仿真时间不一致；本次短时样本出现该项失败，三角波内部换向周期、过冲修正积分和板位刚性通过。不要把所有粗判据都当成严格物理校验。

## 团队协作

从 `main` 新建分支，使用 Pull Request 合并。修改场景参数优先使用个人配置；修改消息定义时，两端重新编译并 source 同一版本。提交功能时附上测试配置、日志窗口、近侧/总命中、发射率及帧率，避免只报一个最高命中率。私有仓库由所有者在 GitHub Settings → Collaborators 邀请队员。
