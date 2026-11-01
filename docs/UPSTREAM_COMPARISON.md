# 与上游的关系

本仓库是南京理工大学江阴校区 Combat 战队的自瞄调试仿真：Windows 机器（RTX 5070 Ti）跑 Daedalus 渲染、物理和云台，Linux 主机跑自瞄。自瞄算法放在另一个仓库，这里只记仿真端和上游的对应关系。

## 对比基准

公开上游是 Blackjack200/bevy_robomaster_simulator。对表时读到的 master 是 `43681255bc5415cdc15797d54fe7478062af42ed`（2026-09-03）。

本机原工程拷过来的时候没有 `.git`，所以没法说清它来自哪个 commit。2025 年 11 月的历史里有布局相近、同样用 Bevy 0.17.2 的单体版本，但好几个提交共用同一批文件，"最像"不等于分叉点。能确定的最早基准是 `src_backup_20261009_233316`，它已经含早期的 ROS 控制和 UDP 真值适配，不是纯净的上游。

分三类写：上游本来就有的、本机改出来的、发布时新增的。上游当前比本机分支功能多，两者不是"升级"关系。

## 能力与结构

| 方面 | 上游 master | 本仓库 |
|---|---|---|
| 目标 | 多目标 RoboMaster 仿真、ROS/Talos 等 | 战队 ROS 图像/自瞄闭环、小陀螺实验 |
| 引擎 | 后续架构已演进，见上游 Cargo.toml | Bevy 0.17.2 / Avian3D 0.4 |
| 源码组织 | scene/systems/config、多个 crates | 单体 main + ros2 + power_rune，另有 spin/shotlog |
| 资源布局 | 根目录 assets，多个机器人/战场模型 | src/assets，5 个编译期嵌入资源 |
| 云台接口 | 说明含 armor_solver/Talos | rune_solver/cmd_gimbal，固定本机消息定义 |
| 场景覆盖 | 已提供能量机关、装甲板和视角控制 | 继承这些功能，默认聚焦固定位置的旋转步兵 |
| 数据集 | 历史路线持续变化 | 保留角点诊断和 writer 类，自动写出没接完 |

## 本机改了什么

| 文件 / 模块 | 行为 | 证据与边界 |
|---|---|---|
| src/spin.rs | 5 种旋转模式、可配置参数、f64 积分、固定随机种子 | 1069 行，19 个既有单元测试；slow 的 7.5 由启动配置覆盖 |
| src/main.rs | 固定敌方 XZ、出生点固定/按键解除、运动写入 Transform 与物理 Rotation | 解决初始场景漂移；实际更新顺序以源码为准 |
| src/ros2/plugin.rs | GimbalCmd 桥接、坐标变换、板中心计算、UDP 快照、采集配置 | 命令度→弧度；板中心取 mesh 顶点质心，不取会重合的节点原点 |
| src/ros2/gt_udp.rs | 独立 JSON 真值，含云台、中心、4 板、spin 状态 | 不占自瞄的 ROS 真值话题；用前要统一时钟与坐标 |
| src/statistic.rs + main.rs | 总命中和近侧命中并列，定期 HITSTAT | near 是命中时板面朝炮口，不等于真机准确率 |
| main.rs 装甲板变换 | 15° 内倾作用于 AM 父节点；支点改为 t_am+t_p+c | 板、灯条和编号一起变换，修掉 AM3/4 安装错位 |
| main.rs 装甲板尺寸 | 代理面归一到 137.80×59.84 mm | 135×56 mm 的尝试已回滚；与算法 135.7×56 模型仍有差异 |
| main.rs 发射/控制 | 冷却 0.05 s；云台限速 6 rad/s；弹丸寿命 2 s；碰撞过滤调整 | 20 发/s 是上限，实际发射率达不到；2 s 不适合长滞空弹道 |
| src/shotlog.rs | req/launch/hit/miss 事件和 shot_id、近侧标记 | SHOT_LOG 门控，不改开火决策；存在 NaN 和重复碰撞等解析边界 |
| ros2/capture.rs、topic.rs | CAM 仪表：回读/编码/发送失败等诊断 | 能区分采集和 DDS 发送问题，不代表解决了 GPU 回读瓶颈 |
| src/util/image.rs | 环境变量配置 JPEG 质量 | 发布配置 q100；JPEG 仍是有损编码 |
| FastDDS XML | maxMessageSize=1400；8 MB socket buffer；沿用旧文件名 | 避免大 RTPS 数据报触发 IP 分片；没设 NIC 白名单 |

拿 2026-10-09 的快照逐字节比，有 **9 个 `.rs` 文件不同或新增**（含新加的 spin/shotlog）。`src/ros2.rs` 的 UDP 模块声明在那个快照里已经有了，所以不算进这个增量，它可能出现在别的日期的同步清单中。差异见 [patch](evidence/changes-since-20261009.patch)，那份文本是拿来读的，不代表能按字节还原；精确版本还是以 commit 和 [SHA256](evidence/source-sha256.json) 为准。

## 回滚掉的、没做的、别人做的

- **FOV 环境变量没落地。** plugin.rs 里垂直 FOV 还是 45° 硬编码。备份目录名里带 fov_env 不代表已经实现。
- **135×56 mm 装甲板已回滚。** 当前是 137.80×59.84 mm，别按备份目录名判断。
- **`/camera_info` 喂给外部算法：** 仿真会发，但 sp_vision_25 从 YAML 取内参，没订阅这个话题。改分辨率要算法端同步。
- **Linux 侧的显示去白边/缩放、pose 开火门、25 s 预热门、高度分组：** 都是外部 C++ 算法改动，没打进本仓库。
- 早期"残余 miss 不可区分""far-hit 是固定物理上限"这类说法，10-14 做时钟对齐复核时有疑问。没有可靠校时和完整样本，这些结论不再沿用，旧蒙特卡洛模型也不算验收依据。
- Linux 主机没登录过，真机入口没移植，也没引入新的自瞄算法。

## 发布时新增的改动

1. 本机活跃的 `.rs` 和 5 个嵌入资源原样保留，SHA256 一致；Cargo 工程放根目录，跟上游根目录放 Cargo 的习惯一致。
2. 收进本机实际在用的 rm_interfaces 消息/服务源码，补齐 `std_msgs` 的 package.xml/CMake 声明，按上游保留 Apache-2.0。
3. 存了 Cargo.lock、Rust 工具链、现有 Pixi manifest/lock，以及官方 ROS 包的 URL 和 SHA256。
4. 新入口支持配置覆盖和自动 MSVC 检测，去掉了启动时强杀进程、必须先有 exe 才能编译、写死本机路径这些限制。
5. Git 里只放能维护的源码和资源，编译好的程序和原生依赖走 Release。默认真值目标改成 loopback，日志级别压掉高频 SIM_GT 刷屏，本机原工程不动。
6. 工具默认路径改成相对仓库；加了环境、测试、证据和协作说明，本机的记忆文件和历史远程登录脚本没上传。

## 版权与协作

根许可证 AGPL-3.0，保留 Blackjack200 / Actor&Thinker 的来源说明。模型是本机继承的资源，来源归属照留，没有当成这次新建的。便携包里带第三方许可证目录。对外分发修改版时继续履行各许可证要求，别把整个仓库改标成 MIT。
