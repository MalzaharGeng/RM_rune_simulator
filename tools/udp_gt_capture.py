# -*- coding: utf-8 -*-
"""
抓取 daedalus 仿真的真值 UDP 流 (JSON), 并打印统计摘要。

用法:
    python tools/udp_gt_capture.py [秒数] [端口]

前置条件: 仿真必须以 GT_UDP_TARGET 指向本机, 例如
    set GT_UDP_TARGET=127.0.0.1:9870
目标必须指向运行本工具的机器。

输出:
    logs/gt.jsonl   每行一帧原始 JSON
    终端打印: 帧数/帧率/关键字段统计
"""
from pathlib import Path
import json
import socket
import sys
import time

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 9870
OUT = sys.argv[3] if len(sys.argv) > 3 else str(Path(__file__).resolve().parents[1] / "logs" / "gt.jsonl")
Path(OUT).parent.mkdir(parents=True, exist_ok=True)


def main() -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", PORT))
    s.settimeout(1.0)

    frames = []
    end = time.time() + SECS
    while time.time() < end:
        try:
            data, addr = s.recvfrom(65535)
            frames.append((time.time(), addr, data.decode("utf-8", "replace")))
        except socket.timeout:
            pass
        except OSError:
            break

    with open(OUT, "w", encoding="utf-8") as f:
        for _, _, d in frames:
            f.write(d.strip() + "\n")

    print("frames=%d  saved=%s" % (len(frames), OUT))
    if not frames:
        print("没抓到任何帧。检查仿真是否带 GT_UDP_TARGET=127.0.0.1:%d 启动。" % PORT)
        return

    wall = frames[-1][0] - frames[0][0]
    print("wall=%.2fs  rate=%.1f Hz" % (wall, len(frames) / wall if wall > 0 else 0))

    def parse(d):
        try:
            return json.loads(d).get("gt", {})
        except Exception:
            return {}

    gt0, gtN = parse(frames[0][2]), parse(frames[-1][2])
    keys = ["gimbal_yaw", "gimbal_pitch", "gimbal_x", "gimbal_y", "gimbal_z"]
    print("\n-- 首帧 / 末帧 --")
    for k in keys:
        if k in gt0:
            print("  %-14s %12.6f  ->  %12.6f" % (k, gt0[k], gtN.get(k, float("nan"))))

    print("\n-- 装甲板 (ROS 系) --")
    for i in range(1, 5):
        p = "enemy_armor_%d_" % i
        if p + "x" in gt0:
            a = [gt0[p + c] for c in "xyz"]
            b = [gtN.get(p + c, float("nan")) for c in "xyz"]
            print("  armor_%d  (%.4f,%.4f,%.4f) -> (%.4f,%.4f,%.4f)" % (i, *a, *b))

    print("\n-- 小陀螺 --")
    for k in ("spin_mode", "spin_angle", "spin_omega", "spin_alpha",
              "spin_target", "spin_phase", "spin_event_t"):
        if k in gtN:
            print("  %-14s %s" % (k, gtN[k]))
    if "spin_angle" in gt0 and "spin_angle" in gtN and wall > 0:
        print("  实测平均角速度 %.3f rad/s (%.3f s/圈)"
              % ((gtN["spin_angle"] - gt0["spin_angle"]) / wall,
                 6.2831853 * wall / (gtN["spin_angle"] - gt0["spin_angle"])
                 if gtN["spin_angle"] != gt0["spin_angle"] else float("inf")))


if __name__ == "__main__":
    main()
