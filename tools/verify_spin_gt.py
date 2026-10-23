# -*- coding: utf-8 -*-
"""
校验敌方步兵「小陀螺」真值是否符合当前模式的预期。

用法:
    python tools/verify_spin_gt.py [jsonl路径] [MIN] [MAX]

默认读 logs/gt.jsonl (由 tools/udp_gt_capture.py 生成)。
MIN / MAX 是配置的角速度区间, 默认 -4 / 4 (与 src/spin.rs 的默认一致)。

按 `spin_mode` 自动分支:
    0 Constant / 1 Regular / 2 Irregular / 3 Slow / 4 Triangle

三角波 (mode=4) 判据:
    A. spin_alpha 恒为 ±alpha_max, **绝不为 0**, 且 |alpha| 全程恒定
    B. spin_phase 恒为 0 (没有保持/匀速段)
    C. alpha_max == (MAX - MIN) / spin_t_acc
    D. spin_omega 始终在 [MIN, MAX] 内, 且两端都到达
    E. 换向周期 == spin_t_acc, 用 spin_event_t 判定, 容差 1e-5 s
       (spin_event_t 是仿真内部记下的精确换向时刻, 不受帧采样相位影响)
    F. 每个半周期精确扫过 (MAX-MIN) rad/s —— **过冲修正**后判定:
           |om[i1]-om[i0]| + over(i0) + over(i1) == MAX-MIN
       其中 over(i) = min(|om[i]-MIN|, |om[i]-MAX|) 是该帧跨过端点后的过冲量。
       状态机在帧中途精确落在端点上、随后同帧朝反向继续走, 所以换向帧的
       omega 已经偏离端点最多一个帧步; 把这段过冲加回去, 扫幅应当精确等于 MAX-MIN。
    F2. **半周期过冲修正角积分 == (MIN+MAX)/2 * spin_t_acc**。
       把每帧角度倒推回"omega 恰好落在端点"的那一刻:
           ang_end(i) = spin_angle[i] - (E_i + om[i])/2 * (over(i)/alpha_max)
       (E_i 是离 om[i] 最近的端点; over(i)/alpha_max 是从端点走到 om[i] 用的时间,
        该段角度增量 = 平均角速度 * 时间)。
       相邻两个换向点的 ang_end 之差, 就是一个完整半周期的 ∫omega dt,
       恒等于 (MIN+MAX)/2 * t_acc。**这条只用仿真内部量, 与 GT 时间戳抖动无关**,
       是负下限(摇摆)情形下唯一过硬的角积分判据 (实测精确到 1e-16)。
    G. spin_omega 真正触及两端: min(omega)-MIN 与 MAX-max(omega) 都要小于
       "0.1 个 95 分位帧步"。这是唯一能排除"提前掉头"的一条 —— 若曲线在 17.5
       就掉头, F 会因为 over 取到 0.5 而仍然凑出 8.0, 但这里会立刻露馅。
       (换向帧的过冲量只作 INFO 打印: 真实积分步长用 Bevy Time::delta, 与 GT
        时间戳不是同一时钟, 卡顿帧上 Time::delta 明显大于时间戳间隔, 拿时间戳
        分位数当上界会误判。)
    H. target∈{MIN,MAX} 且 sign(alpha)=sign(target-omega)

    说明 —— 为什么**不用**"逐帧/短窗口 d(omega)/dt == alpha":
    GT 的 timestamp 是 r2r::Clock 在帧中途采样的系统时钟, 而曲线积分用的是
    帧首的 Time::delta, 两者每帧差 1~2 ms; 高帧率下(170+ Hz)帧间隔本身还在
    2~20 ms 之间抖动。对 omega 做数值微分会把这层抖动放大成 ±15%~30% 的噪声,
    越界率 ~19%, 完全是测量误差而非曲线缺陷。该指标现在只打印, 不作判定;
    真正过硬的等价判据是上面 E + F 两条 (积分意义上的 d(omega)/dt == alpha)。

通用判据 (所有模式):
    无重复帧 / d(spin_angle) 符号与 spin_omega 一致 / 累积 ∫omega dt ≈ Δangle /
    装甲板刚性·不重合·方位角 = spin_angle + 常量
    (min_omega 允许为负; 此时 omega 穿过 0 换向、角度来回摆, 所以只查符号一致性,
     不查单调。累积 ∫omega dt == Δangle 只是**粗判据** (时间戳抖动按 sqrt(N) 随机游走,
     负下限时 ∫omega dt ≈ 0 会被放大); 三角波请以 F2 的"半周期过冲修正角积分"为准。)
"""
from pathlib import Path
import json
import math
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parents[1] / "logs" / "gt.jsonl")
MIN_OMEGA = float(sys.argv[2]) if len(sys.argv) > 2 else -4.0
MAX_OMEGA = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
ALPHA = 120.0           # 非三角波模式的配置角加速度
MIN_DELTA = 3.0
HOLD_MIN, HOLD_MAX = 0.20, 0.50
WARMUP_S = 1.0
TAU = 2 * math.pi

MODE_NAMES = {0: "Constant", 1: "Regular", 2: "Irregular", 3: "Slow", 4: "Triangle"}

ok_all = True


def check(name, cond, detail=""):
    global ok_all
    if not cond:
        ok_all = False
    print("[%s] %-38s %s" % ("PASS" if cond else "FAIL", name, detail))
    return cond


def circ_mean(xs):
    return math.atan2(sum(math.sin(x) for x in xs), sum(math.cos(x) for x in xs))


def circ_resid(xs, ref):
    return [(x - ref + math.pi) % TAU - math.pi for x in xs]


def main():
    rows = []
    with open(PATH, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass

    print("file = %s" % PATH)
    print("frames = %d" % len(rows))
    if len(rows) < 50:
        print("帧数太少, 无法校验。")
        return 1

    gt = [r.get("gt", {}) for r in rows]
    ts = [r.get("timestamp", float("nan")) for r in rows]
    wall = ts[-1] - ts[0]
    print("wall = %.2f s   rate = %.1f Hz" % (wall, len(rows) / wall if wall > 0 else 0))
    warm = set(i for i, t in enumerate(ts) if t - ts[0] < WARMUP_S)
    print("warm-up 段 = 前 %d 帧 (%.2f s), 不参与逐帧判据" % (len(warm), WARMUP_S))
    print("")

    # ---- 模式 ----
    modes = sorted({g.get("spin_mode") for g in gt})
    mode = modes[0] if len(modes) == 1 else None
    check("spin_mode 全程一致", len(modes) == 1,
          "mode=%s (%s)" % (modes, MODE_NAMES.get(mode, "?")))
    print("")

    # ---- 通用 ----
    om = [g.get("spin_omega", float("nan")) for g in gt]
    al = [g.get("spin_alpha", float("nan")) for g in gt]
    ang = [g.get("spin_angle", 0.0) for g in gt]
    d = [ang[i + 1] - ang[i] for i in range(len(ang) - 1)]
    dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    # 帧间隔 95 分位 + 角加速度幅值, 后面算容差要用
    dv = sorted(dts[i] for i in range(len(dts)) if i not in warm and 1e-5 < dts[i] < 0.1)
    dt95 = dv[int(len(dv) * 0.95)] if dv else 0.02
    am = max(abs(a) for a in al) if al else 0.0
    print("        采样 95 分位间隔 = %.4f s   |alpha| = %.3f" % (dt95, am))

    # spin_angle 的增减方向必须与 omega 的符号一致。
    # (min_omega 允许为负; 此时 omega 持续穿过 0 换向, 角度来回摆, 不能再要求单调。)
    # 只在相邻两帧 omega 同号且明显非 0 时判定, 避开 omega 穿过 0 的那一帧。
    sign_bad = []
    for i in range(len(d)):
        w0, w1 = om[i], om[i + 1]
        if w0 > 0.2 and w1 > 0.2 and d[i] <= 0:
            sign_bad.append((i, round(w0, 3), round(d[i], 6)))
        elif w0 < -0.2 and w1 < -0.2 and d[i] >= 0:
            sign_bad.append((i, round(w0, 3), round(d[i], 6)))
    check("d(spin_angle) 符号 == sign(spin_omega)", not sign_bad,
          "违规帧数=%d %s" % (len(sign_bad), sign_bad[:3]))

    # 重复帧: 角度逐位不变、但该帧 omega 明显非 0 (真值读到了上一帧的旧角度)
    dup = [i for i in range(len(d))
           if dts[i] > 1e-4 and abs(d[i]) < 1e-12 and abs(om[i]) > 0.1]
    check("无重复帧 (d_angle=0 但 omega≠0)", not dup,
          "重复帧数=%d (占 %.1f%%)" % (len(dup), 100.0 * len(dup) / len(d)))

    # 累积 ∫omega dt 与 Δangle。尺度取 ∫|omega|dt。
    # ⚠️ 这条只是**粗判据**: dt_stamp 与仿真内部的 dt 不是同一个时钟, 抖动会按 sqrt(N)
    #    累积成随机游走 (实测 ~0.3 rad)。单调自转时 ∫omega dt 很大, 这点误差可忽略;
    #    omega 有正有负时 ∫omega dt 接近 0, 相对误差会被放大 —— 那种情形请以三角波的
    #    "半周期过冲修正角积分" 为准 (精确到 1e-16)。
    S = 0.0
    S_abs = 0.0
    for i in range(len(gt) - 1):
        if dts[i] > 0.2:
            continue
        S += (om[i] + om[i + 1]) * 0.5 * dts[i]
        S_abs += (abs(om[i]) + abs(om[i + 1])) * 0.5 * dts[i]
    dA = ang[-1] - ang[0]
    tol_abs = max(0.005 * S_abs, 0.02 * am * dt95 * math.sqrt(len(gt)))
    check("累积 int(omega)dt ≈ Δangle", abs(S - dA) <= tol_abs,
          "∫ωdt=%.3f  ∫|ω|dt=%.3f  Δangle=%.3f  |差|=%.3f  容差=%.3f" % (
              S, S_abs, dA, abs(S - dA), tol_abs))

    # ---- 模式相关 ----
    if mode == 4:
        print("")
        print("---- 三角波专用判据 ----")
        amin = min(abs(a) for a in al)
        amax = max(abs(a) for a in al)
        am = amax
        check("spin_alpha 绝不为 0 (无匀速段)", amin > 1e-9, "|alpha| min=%.6f" % amin)
        check("|spin_alpha| 全程恒定", amax - amin < 1e-6,
              "alpha_max=%.6f  spread=%.2e" % (am, amax - amin))
        check("spin_alpha 只取 ±alpha_max",
              all(abs(abs(a) - am) < 1e-6 for a in al),
              "取值集合=%s" % sorted(set(round(a, 6) for a in al)))
        check("spin_alpha 正负交替", min(al) < 0 < max(al),
              "min=%.3f max=%.3f" % (min(al), max(al)))

        ph = sorted({g.get("spin_phase") for g in gt})
        check("spin_phase 恒为 0 (无保持段)", ph == [0], "observed=%s" % ph)

        t_acc = gt[-1].get("spin_t_acc")
        if t_acc:
            expect = (MAX_OMEGA - MIN_OMEGA) / t_acc
            check("alpha_max == (MAX-MIN)/spin_t_acc", abs(am - expect) / expect < 1e-6,
                  "实测 %.6f  期望 %.6f  (spin_t_acc=%s)" % (am, expect, t_acc))
        else:
            check("真值含 spin_t_acc", False, "缺字段")

        check("spin_omega in [%.0f, %.0f]" % (MIN_OMEGA, MAX_OMEGA),
              min(om) >= MIN_OMEGA - 1e-6 and max(om) <= MAX_OMEGA + 1e-6,
              "min=%.6f max=%.6f" % (min(om), max(om)))

        # 端点是否**真正**被触及。这是 G' 判据, 也是唯一能排除"提前掉头"的一条:
        # 若曲线在 17.5 就掉头, F 会凑出 8.0 (over 取到 0.5), 而这里会露馅。
        # 采样点离端点的最近距离 ~ U[0, dt], N 次换向取最小后约为 dt/N, 所以门槛
        # 取"0.1 个 95 分位帧步"就足够宽, 又远小于任何有意义的提前掉头。
        eps = max(am * dt95 * 0.10, 1e-3)
        check("omega 真正触及两端 (逼近 < 0.1 帧步)",
              min(om) - MIN_OMEGA <= eps and MAX_OMEGA - max(om) <= eps,
              "下探到 %.6f (距 MIN %.5f), 上冲到 %.6f (距 MAX %.5f), 门槛 %.5f" % (
                  min(om), min(om) - MIN_OMEGA, max(om), MAX_OMEGA - max(om), eps))

        # 换向时刻 = spin_event_t 变化的帧。spin_event_t 是仿真内部记下的
        # 精确换向时刻 (sim_t - remaining), 不受帧采样相位影响。
        bounds = [0]
        last = gt[0].get("spin_event_t")
        for i in range(1, len(gt)):
            e = gt[i].get("spin_event_t")
            if e != last:
                bounds.append(i)
                last = e
        bounds.append(len(gt))
        flips = bounds[1:-1]          # 换向帧索引
        evs = [gt[b].get("spin_event_t") for b in flips]

        # E. 换向周期精确等于 spin_t_acc
        per = [evs[i + 1] - evs[i] for i in range(len(evs) - 1)]
        if t_acc and per:
            worst_p = max(abs(p - t_acc) for p in per)
            check("换向周期 == spin_t_acc (精确)", worst_p < 1e-5,
                  "n=%d  实测 %.6f~%.6f s  期望 %.4f  最大偏差=%.2e s" % (
                      len(per), min(per), max(per), t_acc, worst_p))
        else:
            check("换向周期 == spin_t_acc (精确)", False, "无换向事件")

        # F. 过冲修正后, 每个半周期精确扫过 MAX-MIN
        def over(i):
            return min(abs(om[i] - MIN_OMEGA), abs(om[i] - MAX_OMEGA))

        segs = []
        for k in range(len(flips) - 1):
            i0, i1 = flips[k], flips[k + 1]
            segs.append(abs(om[i1] - om[i0]) + over(i0) + over(i1))
        if segs:
            worst_s = max(abs(s - (MAX_OMEGA - MIN_OMEGA)) for s in segs)
            raw_med = sorted(abs(om[flips[k + 1]] - om[flips[k]])
                             for k in range(len(flips) - 1))[len(segs) // 2]
            check("每半周期精确扫过 MAX-MIN (过冲修正)", worst_s < 1e-3,
                  "n=%d  最大偏差=%.2e rad/s  (未修正中位扫幅 %.4f)" % (
                      len(segs), worst_s, raw_med))
        else:
            check("每半周期精确扫过 MAX-MIN (过冲修正)", False, "换向区间不足")

        # F2. 半周期过冲修正角积分 == (MIN+MAX)/2 * t_acc
        #     把每帧角度倒推回 "omega 恰好落在端点" 的那一刻:
        #         ang_end(i) = spin_angle[i] - (E_i + om[i])/2 * (over(i)/alpha_max)
        #     相邻两个换向点的 ang_end 之差 = 一个完整半周期的 ∫omega dt
        #         = (MIN+MAX)/2 * t_acc
        #     这条只用仿真内部量 (spin_angle / spin_omega / alpha_max), 与 GT 时间戳
        #     抖动完全无关; 负下限(摇摆)时它是唯一过硬的角积分判据。
        if t_acc and am > 0 and len(flips) >= 2:
            def ang_end(i):
                e = MIN_OMEGA if abs(om[i] - MIN_OMEGA) < abs(om[i] - MAX_OMEGA) else MAX_OMEGA
                return ang[i] - (e + om[i]) * 0.5 * (over(i) / am)

            expect_a = (MIN_OMEGA + MAX_OMEGA) * 0.5 * t_acc
            dsa = [ang_end(flips[k + 1]) - ang_end(flips[k]) for k in range(len(flips) - 1)]
            worst_a = max(abs(s - expect_a) for s in dsa)
            check("半周期过冲修正角积分 == (MIN+MAX)/2*t_acc", worst_a < 1e-6,
                  "n=%d  实测 %.3e~%.3e rad  期望 %.3e  最大偏差=%.2e" % (
                      len(dsa), min(dsa), max(dsa), expect_a, worst_a))
        else:
            check("半周期过冲修正角积分 == (MIN+MAX)/2*t_acc", False, "无换向事件")

        # [INFO] 换向帧过冲量。理论上 over <= alpha_max * (该帧真实积分步长),
        # 但真实步长用的是 Bevy 的 Time::delta, 与 GT 时间戳并非同一时钟:
        # 卡顿帧上 Time::delta 会明显大于时间戳间隔, 所以拿时间戳分位数当上界
        # 会误判。这里只打印, 不作判定; 端点是否被触及由上面 G' 保证。
        ov = [over(b) for b in flips]
        print("        [INFO] 换向帧过冲 max=%.4f rad/s (即 %.2f ms 的 alpha), "
              "中位 %.4f —— 仅参考" % (
                  max(ov) if ov else float("nan"),
                  1000.0 * (max(ov) / am) if ov else float("nan"),
                  sorted(ov)[len(ov) // 2] if ov else float("nan")))

        # [INFO] 窗口平均斜率 —— 只打印, 不作判定。
        # 逐帧数值微分被时间戳抖动淹没 (见文件头说明), 故降级为参考量。
        W = 5
        tol_s = 0.15 * am
        bad_w, n_w, worst_ws = 0, 0, 0.0
        for i in range(len(gt) - W):
            if i in warm or (i + W) in warm:
                continue
            span = ts[i + W] - ts[i]
            if span < 1e-4 or span > 0.2:
                continue
            n_w += 1
            v = (om[i + W] - om[i]) / span
            lo = min(al[i:i + W + 1]) - tol_s
            hi = max(al[i:i + W + 1]) + tol_s
            if not (lo <= v <= hi):
                bad_w += 1
                worst_ws = max(worst_ws, max(lo - v, v - hi))
        print("        [INFO] 窗口(%d帧)平均斜率越界 %.2f%% (%d/%d), 容差±%.1f rad/s², "
              "最大超出 %.2f —— 受时间戳抖动限制, 仅参考" % (
                  W, 100.0 * bad_w / max(n_w, 1), bad_w, n_w, tol_s, worst_ws))

        # H. 状态机逻辑: target 只取两端, 且 alpha 符号必须指向 target
        bad_dir = []
        for i, g in enumerate(gt):
            tgt, w, a = g.get("spin_target"), g.get("spin_omega"), g.get("spin_alpha")
            if tgt not in (MIN_OMEGA, MAX_OMEGA) or abs(a) < 1e-9:
                bad_dir.append((i, "target/alpha"))
            elif (a > 0) != (tgt > w):
                bad_dir.append((i, "sign"))
        check("target∈{MIN,MAX} 且 sign(alpha)=sign(target-omega)", not bad_dir,
              "违规帧数=%d %s" % (len(bad_dir), bad_dir[:3]))
    else:
        print("")
        print("---- 变速/匀速模式判据 ----")
        check("spin_omega in [%.0f, %.0f]" % (MIN_OMEGA, MAX_OMEGA),
              min(om) >= MIN_OMEGA - 1e-6 and max(om) <= MAX_OMEGA + 1e-6,
              "min=%.4f max=%.4f mean=%.4f" % (min(om), max(om), sum(om) / len(om)))
        als = sorted({round(a, 6) for a in al})
        check("spin_alpha in {0, +/-%.0f}" % ALPHA,
              all(abs(a) in (0.0, ALPHA) for a in als), "observed=%s" % als)
        bad = [i for i, g in enumerate(gt)
               if g.get("spin_phase") not in (0, 1)
               or (g.get("spin_phase") == 0) != (abs(g.get("spin_alpha", 0.0)) > 0)]
        check("spin_phase <-> alpha 一致", not bad, "bad=%d" % len(bad))
        tg = [g.get("spin_target", 0.0) for g in gt]
        sw = [i for i in range(1, len(tg)) if abs(tg[i] - tg[i - 1]) > 1e-6]
        steps = [abs(tg[i] - tg[i - 1]) for i in sw]
        check("目标切换间隔 >= min_delta(%.0f)" % MIN_DELTA,
              all(s >= MIN_DELTA - 1e-6 for s in steps),
              "切换=%d 次 最小步长=%.4f" % (len(sw), min(steps) if steps else float("nan")))
        segs = []
        cur_ev, cur_ph = gt[0].get("spin_event_t"), gt[0].get("spin_phase")
        for g in gt[1:]:
            if g.get("spin_event_t") != cur_ev:
                segs.append((cur_ph, g["spin_event_t"] - cur_ev))
                cur_ev, cur_ph = g["spin_event_t"], g["spin_phase"]
        holds = [dur for ph, dur in segs if ph == 1]
        check("保持段时长 in [%.2f, %.2f]s" % (HOLD_MIN, HOLD_MAX),
              bool(holds) and all(HOLD_MIN - 0.01 <= h <= HOLD_MAX + 0.01 for h in holds),
              "n=%d min=%.4f max=%.4f" % (
                  len(holds), min(holds) if holds else float("nan"),
                  max(holds) if holds else float("nan")))

    # ---- 装甲板 ----
    print("")
    print("---- 装甲板 ----")
    cen = [(g.get("enemy_center_x"), g.get("enemy_center_y"), g.get("enemy_center_z")) for g in gt]
    radii = {}
    for i in range(1, 5):
        k = "enemy_armor_%d_" % i
        rs = []
        for g, c in zip(gt, cen):
            if c[0] is None or k + "x" not in g:
                continue
            a = (g[k + "x"], g[k + "y"], g[k + "z"])
            rs.append(math.sqrt(sum((a[j] - c[j]) ** 2 for j in range(3))))
        if rs:
            radii[i] = (min(rs), max(rs))
    check("装甲板相对车体中心刚性", all(mx - mn < 0.002 for mn, mx in radii.values()),
          "; ".join("A%d %.4f~%.4f" % (i, mn, mx) for i, (mn, mx) in sorted(radii.items())))
    min_pair = 1e9
    for g in gt:
        pts = []
        for i in range(1, 5):
            k = "enemy_armor_%d_" % i
            if k + "x" in g:
                pts.append((g[k + "x"], g[k + "y"], g[k + "z"]))
        for x in range(len(pts)):
            for y in range(x + 1, len(pts)):
                min_pair = min(min_pair, math.dist(pts[x], pts[y]))
    check("4 块装甲板两两不重合", min_pair > 0.05, "最近间距=%.4f m" % min_pair)

    azis, refs = [], []
    for g in gt:
        if g.get("enemy_center_x") is None or "enemy_armor_1_x" not in g:
            continue
        dx = g["enemy_armor_1_x"] - g["enemy_center_x"]
        dy = g["enemy_armor_1_y"] - g["enemy_center_y"]
        azis.append(math.atan2(dy, dx))
        refs.append(g["spin_angle"])
    diff = [azis[i] - refs[i] for i in range(len(azis))]
    mu = circ_mean(diff)
    r = circ_resid(diff, mu)
    std = math.sqrt(sum(x * x for x in r) / len(r))
    mxres = max(abs(x) for x in r)
    check("装甲板方位角 = spin_angle + 常量", mxres < 0.06,
          "偏移=%.4f rad (%.1f deg)  残差 std=%.4f  max=%.4f rad (%.2f deg)" % (
              mu, math.degrees(mu), std, mxres, math.degrees(mxres)))
    bins = {}
    for i in range(len(azis)):
        bins.setdefault(int((refs[i] % TAU) / TAU * 8), []).append(r[i])
    bm = [sum(v) / len(v) for _, v in sorted(bins.items())]
    check("残差与旋转角无关", max(bm) - min(bm) < 0.03,
          "8 个角度分箱残差均值极差=%.4f rad" % (max(bm) - min(bm)))

    print("")
    print("== 结论: %s ==" % ("全部通过" if ok_all else "存在不通过项"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
