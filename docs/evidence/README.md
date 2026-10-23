# 验证证据与使用边界

## 本次执行（2026-10-23）

- 当前 src/spin.rs 直接使用本机已编译 Bevy 依赖重新构建并执行：19 passed / 0 failed。为避免改写原工程 target，测试二进制输出到独立工作目录；没有声称执行完整 cargo test 或重新编译整个仿真器。
- 运行包依赖闭包：316 个 DLL，PE 导入/延迟导入未发现未解析的非系统依赖；包含后续动态加载所需的 ROS 类型支持及 FastDDS 实现。
- PATH 仅包含运行包 bin 和 Windows 系统目录的隔离进程：slow / triangle 各 350 帧完整 UDP 真值；未使用本机 D 盘 ROS/Pixi 路径解析运行库。
- slow 角速度始终 7.5 rad/s；triangle 角速度范围约 [-3.99416,3.99905] rad/s，角加速度绝对值为 16 rad/s²。
- 在发布目录重新构建 rm_interfaces，成功生成全部消息 DLL；安装脚本显式使用 Pixi Python，修复首次误用系统 Python 的缺少 em 错误。
- 实际 ROS 接收图像、CameraInfo、gimbal_pose，发布 fire_advice=False 的云台命令。最终计数与姿态变化见 current-ros-smoke.txt。
- PowerShell 文件静态解析无错误；所有仿真 .rs 和嵌入资源逐字节对照源文件，SHA256 见 source-sha256.json。

## 未通过与未覆盖

旧 verify_spin_gt.py 的系统时间累积积分粗判据，在本次 350 帧/约 2.24 s 的三角波样本上失败：积分约 -0.679 rad，实际角差约 0.051 rad。其时间来自 ROS 系统时钟，运动使用 Bevy Time；短时启动和卡顿使两者不应直接等同。三角波专用的精确半周期、过冲修正积分、速度边界以及装甲板刚性/不重合判据通过。保留工具原有判据并记录限制，没有为了全绿改写仿真或删除失败结果。

云台测试早期以第一帧姿态为唯一基准，出现已到目标姿态导致变化为零的情况；最终工具在首次收到姿态后发送 -5° 再 +5°，核查有实际响应且全程不开火。

未重新执行整个 Rust 仿真器的 cargo build/test，未在另一台全新 Windows 从零安装，未开展 Linux 跨机自瞄或真机试射。本次便携包来自 2026-10-13 的本机已有可执行文件，SHA256 在 runtime-build.json 中记录。历史成功编译只能证明当时构建，不替代新机器验收。

## 历史结果

history/ 保存从用户指定日志筛选的编译结束/EXITCODE/HITSTAT 行，去掉 ANSI 控制码；不是伪造 Git 提交历史。_cargo_build_camdiag 的 EXITCODE=101 后有 camdiag2 的成功记录。旧单元测试日志为特殊 UTF-16 格式，本次以重新执行的 current-spin-tests.txt 为准。

warmup 系列最后一条 HITSTAT 的总/近侧命中百分比因统计时刻和窗口与当时报告略有不同。例如 FINAL 的最后快照是 1729 发、1700 次命中、1637 次近侧命中（98.32% / 94.68%）。末尾仍在飞的弹丸及多次碰撞会影响比值；不将该快照等同逐发去重后的终局命中率。

history/_warm_P07W0*、_warm_P07W25* 与 FINAL 涉及 Linux 自瞄开火参数，算法实现不在本仓库。_ab_run_off/on 与 _aim_b* 已作为原始审查资料核对存在；不同阶段有多次结论更正，本仓库不复述未经可靠时钟对齐的因果结论。完整原始日志、账号信息、内网脚本和记忆记录保留本地，不上传。

changes-since-20261009.patch 用于阅读，与本机最早快照比较；不代表已知精确上游分叉点。source-sha256.json 提供复制源文件的校验，upstream-file-comparison.json 是相对当前公开 master 的路径/字节对比。
