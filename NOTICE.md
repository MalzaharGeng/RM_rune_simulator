# 来源与第三方说明

本仓库的战队适配与维护：南京理工大学江阴校区 Combat 战队（维护者 MalzaharGeng）。用途为 Windows GPU 仿真与 Linux 自瞄主机之间的算法调试和闭环联调。自瞄算法计划在独立仓库发布，链接待补充。

仿真器派生自 Blackjack200 / Actor&Thinker 的 Daedalus / bevy_robomaster_simulator：
https://github.com/Blackjack200/bevy_robomaster_simulator
https://bbs.robomaster.com/article/1887395?source=4

根 LICENSE 为继承的 GNU AGPL v3。rm_interfaces 的消息源来自原本机 ROS overlay，与上游消息包同源；保留上游 Apache-2.0 许可证，原作者信息见其 package.xml。模型、音效沿用本机继承的资源和原有来源，原始作品的版权不属于这次整理。

运行包 licenses/ 目录包含 ROS 包描述和可取得的许可证、Rust crate 许可证、原生依赖许可证及清单。Windows 系统 DLL 不随包复制。Microsoft VC Runtime 的再分发条款随运行包保留。
