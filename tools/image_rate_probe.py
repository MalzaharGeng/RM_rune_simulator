# -*- coding: utf-8 -*-
"""
测量 daedalus 仿真图像话题 (默认 /image_compressed) 的实际发布帧率与间隔抖动。

用法:
    python tools/image_rate_probe.py [话题] [秒数]

为什么不用 `ros2 topic hz`:
    本机 ros2 CLI 的 daemon 起不来 (discovery 会挂住), 必须用 --no-daemon,
    但 `ros2 topic hz` 不接受 --no-daemon。所以直接用 rclpy 订阅。

QoS: 用 BEST_EFFORT 订阅 —— 它与 RELIABLE / BEST_EFFORT 两种发布端都兼容
     (反之 BEST_EFFORT 发布 + RELIABLE 订阅 是不兼容的, 会静默收不到)。

需要已加载 ROS 2 Jazzy 的 Python/rclpy 环境，见 docs/ENVIRONMENT.md。
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage

TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_compressed"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0


def jpeg_size(data: bytes) -> tuple[int, int]:
    """从 JPEG 字节流里读 SOF 段的宽高 (不依赖 PIL)。"""
    i, n = 2, len(data)
    while i + 9 < n:
        if data[i] != 0xFF:
            i += 1
            continue
        m = data[i + 1]
        if m in (0xD8, 0xD9) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        seglen = (data[i + 2] << 8) | data[i + 3]
        if 0xC0 <= m <= 0xCF and m not in (0xC4, 0xC8, 0xCC):
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return w, h
        i += 2 + seglen
    return 0, 0


class Probe(Node):
    def __init__(self, topic: str):
        super().__init__("image_rate_probe")
        # 只挂一个 BEST_EFFORT 订阅: DDS 的兼容规则是
        #   RELIABLE 发 + BEST_EFFORT 收 = 兼容
        #   BEST_EFFORT 发 + BEST_EFFORT 收 = 兼容
        # 所以 BEST_EFFORT 订阅对两种发布端都收得到。
        # ⚠️ 千万别同时挂 BEST_EFFORT + RELIABLE 两个订阅: 两个都会匹配上,
        #    每条消息会被投递两次, 统计出来的帧率会翻倍。
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(CompressedImage, topic, self.cb, qos)
        self.topic = topic
        self.arrivals: list[float] = []
        self.stamps: list[float] = []
        self.sizes: list[int] = []
        self.width = self.height = 0
        self.fmt = ""
        self.frame_id = ""

    def cb(self, msg: CompressedImage) -> None:
        self.arrivals.append(time.time())
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.sizes.append(len(msg.data))
        if self.width == 0:
            self.width, self.height = jpeg_size(bytes(msg.data))
        self.fmt = msg.format or self.fmt
        self.frame_id = msg.header.frame_id or self.frame_id


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))]


def main() -> int:
    rclpy.init()
    node = Probe(TOPIC)
    print("topic = %s   采样 %.1f s" % (TOPIC, SECS))
    t_end = time.time() + SECS
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    n = len(node.arrivals)
    node.destroy_node()
    rclpy.shutdown()

    if n < 3:
        print("只收到 %d 条消息 —— 检查仿真是否在跑、DOMAIN_ID/RMW 是否一致。" % n)
        return 1

    a = node.arrivals
    wall = a[-1] - a[0]
    dt = [a[i + 1] - a[i] for i in range(len(a) - 1)]
    print("")
    print("收到          %d 条 / %.3f s" % (n, wall))
    print("平均帧率      %.2f Hz   (%.2f ms/帧)" % (n / wall, 1000.0 * wall / n))
    print("间隔 dt       min %.2f  p50 %.2f  p95 %.2f  max %.2f ms"
          % (1000 * min(dt), 1000 * pct(dt, 0.5), 1000 * pct(dt, 0.95), 1000 * max(dt)))
    print("间隔 std      %.2f ms" % (1000 * statistics.pstdev(dt)))
    print("")
    print("JPEG 大小     min %d  mean %.0f  max %d 字节  (总 %.2f MB, 合 %.1f Mbps)"
          % (min(node.sizes), statistics.mean(node.sizes), max(node.sizes),
             sum(node.sizes) / 1e6, 8.0 * sum(node.sizes) / wall / 1e6))
    print("图像          %dx%d  format=%s  frame_id=%s"
          % (node.width, node.height, node.fmt, node.frame_id))

    if len(node.stamps) >= 3:
        st = node.stamps
        sd = [st[i + 1] - st[i] for i in range(len(st) - 1)]
        sd = [d for d in sd if 0 < d < 1.0]
        if sd:
            print("")
            print("消息头 stamp 间隔 (仿真侧, 不含网络抖动)")
            print("              min %.2f  p50 %.2f  p95 %.2f  max %.2f ms"
                  % (1000 * min(sd), 1000 * pct(sd, 0.5), 1000 * pct(sd, 0.95), 1000 * max(sd)))
            print("              => %.2f Hz" % (1.0 / statistics.mean(sd)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
