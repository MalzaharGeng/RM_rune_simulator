# 验证证据与使用边界

## 2026-10-23 跑过的东西

- src/spin.rs 用本机已编译的 Bevy 依赖单独重建并执行：19 passed / 0 failed。测试二进制输出到独立工作目录，没动原工程的 target；完整 `cargo test` 和整个仿真器的重编没有做。
- 运行包依赖闭包：316 个 DLL，PE 导入/延迟导入里没有未解析的非系统依赖，后续动态加载要用的 ROS 类型支持和 FastDDS 实现都在。
- PATH 只含运行包 bin 和 Windows 系统目录的隔离进程：slow / triangle 各 350 帧完整 UDP 真值，没走本机 D 盘的 ROS/Pixi 路径解析运行库。
- slow 角速度全程 7.5 rad/s；triangle 角速度范围约 [-3.99416, 3.99905] rad/s，角加速度绝对值 16 rad/s²。
- 在发布目录重建 rm_interfaces，消息 DLL 全部生成。安装脚本显式用 Pixi Python，修掉了第一次误用系统 Python 导致的缺 empy 报错。
- 实际 ROS 收到图像、CameraInfo、gimbal_pose，发布了 fire_advice=False 的云台命令。计数与姿态变化见 current-ros-smoke.txt。
- PowerShell 文件静态解析无错误；仿真 `.rs` 和嵌入资源逐字节对照源文件，SHA256 见 source-sha256.json。

运行包内 exe 与 ZIP 的 SHA256 在 `runtime-build.json` 和 `env/runtime.json` 里，与 Release v2026.10.23 一致；2026-10-31 又把活跃源码、模型/音效、Cargo 依赖和 FastDDS 配置对了一遍，仍然是同一版。

## 没过的和没做的

旧 `verify_spin_gt.py` 的"累积 int(omega)dt"判据用的是系统时间，运动积分用 Bevy Time。350 帧、约 2.24 s 的三角波样本上这项不过：积分约 -0.679 rad，实际角差约 0.051 rad；启动和卡顿时两者本来就不该直接等同。同一份样本里，三角波专用的半周期、过冲修正积分、速度边界，以及装甲板刚性/不重合判据都过了。工具原有判据保留，限制写在记录里，没有为了全绿去改仿真或删掉失败结果。

云台测试早期拿第一帧姿态当唯一基准，出现过已经到目标姿态、变化量为零的情况；后来改成首次收到姿态后先发 -5° 再 +5°，确认有响应且全程不开火。

整个 Rust 仿真器没有重新 `cargo build/test`，没在另一台全新 Windows 上从零装一遍，Linux 跨机自瞄和真机试射也没做。便携包用的是 2026-10-13 本机已有的可执行文件，SHA256 记在 runtime-build.json。历史编译成功只说明当时能编，不等于新机器验收通过。

## 历史结果

`history/` 里是从日志里筛出来的编译结束 / EXITCODE / HITSTAT 行，去掉了 ANSI 控制码，不是编造的 Git 历史。`_cargo_build_camdiag` 的 EXITCODE=101 之后有 camdiag2 的成功记录。旧单元测试日志是特殊的 UTF-16 格式，以重新执行的 current-spin-tests.txt 为准。

warmup 系列末尾那条 HITSTAT 的总/近侧命中百分比，跟当时的统计时刻和窗口略有出入。比如 FINAL 最后一帧快照是 1729 发、1700 次命中、1637 次近侧命中（98.32% / 94.68%）。收尾时还有弹丸在飞，加上重复碰撞，这个快照不等于逐发去重后的终局命中率。

`history/_warm_P07W0*`、`_warm_P07W25*` 和 FINAL 涉及的是 Linux 自瞄的开火参数，算法不在本仓库。`_ab_run_off/on` 和 `_aim_b*` 作为原始审查资料核对过存在；这几个阶段有多次结论更正，时钟没对齐的因果关系这里不重复。完整原始日志、账号信息、内网脚本和记忆记录留在本地，没上传。

changes-since-20261009.patch 用来阅读，跟本机最早的快照比，不代表已知的精确上游分叉点。source-sha256.json 是复制源文件时的校验，upstream-file-comparison.json 是相对当前公开 master 的路径/字节对比。
