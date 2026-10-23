# Combat 战队 Windows / Linux 双机联调

本仿真用于南京理工大学江阴校区 Combat 战队的自瞄调试。Windows 本机用 RTX 5070 Ti 运行 Daedalus，Linux 主机运行战队自瞄算法。自瞄部分将上传到独立仓库，发布后在主 README 补充链接；本文记录仿真端与双方接口的连接方式。

## 两端分工

| 端 | 执行任务 | 对端依赖 |
|---|---|---|
| Windows / RTX 5070 Ti | 场景与图像渲染、云台/弹丸仿真、图像/内参/姿态发布、接收控制 | 需要 Linux 自瞄节点提供云台和开火建议 |
| Linux / Ubuntu 24.04.3 | 图像检测、跟踪、预测、规划，发布控制；可运行真值分析工具 | 需要 Windows 图像和姿态、匹配的消息定义和相机参数 |

Linux 主机资料记录 ROS 2 Jazzy、OpenVINO 2024.6.0、约 31 GiB 内存，只有 Intel 集显，所列算法配置使用 CPU 推理。RTX 5070 Ti 属于 Windows 仿真机，不是 Linux 推理设备；其他队员可用兼容显卡替代当前开发机型号。

## Windows 准备

按 [README](../README.md) 克隆仓库并安装运行包，或者按 [开发环境说明](ENVIRONMENT.md) 编译。运行入口为 start_rune_simulator.bat；通过启动日志中的 AdapterInfo 和 NVIDIA 工具确认仿真使用预期显卡。

复制 config/local.example.json 到 config/local.json，配置两端共用的 domain，并将真值发往 Linux 实际可达的局域网地址，例如：

```json
{
  "ROS_DOMAIN_ID": "25",
  "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
  "GT_UDP_TARGET": "192.168.1.100:9870",
  "SPIN_MODE": "slow",
  "SPIN_SLOW_OMEGA": "7.5"
}
```

上面的 IP 为示例，需替换成 Linux 主机当前地址。发布默认值 127.0.0.1:9870 用于本机检查，不会自动把 UDP 真值转发到 Linux。ROS 图像/控制使用 DDS 发现，不通过 GT_UDP_TARGET 发送。

## Linux 准备

先加载已有 ROS 2 Jazzy 环境：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

两端必须使用一致的 rm_interfaces 定义。如果算法工作区已有该包，应核对后复用；若需要单独构建，可以在 Linux 获取本仓库后创建 overlay：

```bash
mkdir -p ~/combat_sim_ws
cd ~/combat_sim_ws
# 替换为 Linux 上克隆仓库的实际绝对路径。
colcon build --base-paths /absolute/path/RM_rune_simulator/rm_interfaces --packages-select rm_interfaces
source install/setup.bash
```

新终端仍需 source ROS 和消息 overlay，并设置相同的 domain/RMW。自瞄算法的安装、权重和启动命令由后续算法仓库提供；本仓库的 Windows 便携包不包含这些部分。

## 启动与检查

1. 确认 Windows 和 Linux 在可互通局域网，两端 ROS_DOMAIN_ID 相同，网络与防火墙允许 FastDDS 发现和数据传输。
2. Windows 启动仿真；Linux 加载 ROS、消息 overlay 和算法环境，检查是否可发现图像、位姿及控制话题，再启动自瞄。
3. 若要记录诊断真值，在 Linux 执行 `python3 tools/udp_gt_capture.py 20 9870 logs/gt.jsonl`；该工具可从本仓库使用，运行位置与 GT_UDP_TARGET 必须对应。
4. Linux 用 `python3 tools/image_rate_probe.py /image_compressed 20` 检查实际收图。该工具需要已有 rclpy 环境；仿真窗口 FPS 不等于算法接收帧率。
5. 接入算法时核对 `/rune_solver/cmd_gimbal` 的 GimbalCmd 类型和 QoS，确认实际云台姿态随指令变化，再按 [测试手册](TESTING.md) 进行开火实验。

| 数据方向 | 数据 | 关键约定 |
|---|---|---|
| Windows → Linux / ROS 2 | `/image_compressed`、`/camera_info`、`/gimbal_pose`、其他位姿/TF | 默认图像 960×720、JPEG q100；采集垂直 FOV 45° |
| Linux → Windows / ROS 2 | `/rune_solver/cmd_gimbal` | GimbalCmd 的 yaw/pitch/yaw_diff/pitch_diff 为度，distance 为米，fire_advice 为开火建议 |
| Windows → Linux / UDP | timestamp + gt，端口默认 9870 | 诊断真值的角度为弧度，供对照分析；与 ROS 控制及图像通道独立 |

现有自瞄接入记录从 YAML 读取内参，并不自动消费 CameraInfo。960×720 下理想内参 fx=fy≈869.1169、cx=480、cy=360；改变采集尺寸时要同步检查算法内参和 ROI。计算真值误差前还需统一坐标、角度单位和时钟。

## 文档与配置的管理

本机 IP 和安装路径放在被 Git 忽略的 config/local.json。登录凭据、个人主机名和原始主机信息文件留在本地；仓库只记录部署所需的硬件/软件与接口信息。Linux 资料来自战队提供的主机文档，文中启动流程尚需配合实际自瞄仓库进行跨机验收，不表示本次已重新登录 Linux 完成联调。
