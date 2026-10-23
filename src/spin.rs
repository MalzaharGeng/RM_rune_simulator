//! 敌方步兵「小陀螺」运动曲线
//!
//! 两种用途:
//! - **慢速固定旋转** ([`SpinMode::Slow`]): 原地以恒定 3.0 rad/s 匀速自转,
//!   等价于改造前的 `Quat::from_rotation_y(3.0 * t)`。
//! - **变速小陀螺** ([`SpinMode::Triangle`] / [`SpinMode::Constant`] / [`SpinMode::Regular`]
//!   / [`SpinMode::Irregular`]): 高速自转并做变速, 用于测试我方自瞄的跟踪 / 运动预测 / 装甲板切换。
//!
//! [`SpinMode::Triangle`] 是确定性理想三角波:
//! ```text
//! omega(t): 在 [min_omega, max_omega] 之间线性往复 (三角波)
//! alpha(t): 纯方波, 恒为 ±alpha_max, 没有 0 (匀速) 段
//! alpha_max = (max_omega - min_omega) / t_acc
//! 周期     = 2 * t_acc
//! ```
//! 关系满足 `omega = ∫ alpha dt`, 且 `omega` 到端点时 `alpha` 立即反号。
//!
//! 注意: `min_omega` 允许为负。此时 `omega` 会持续穿过 0 换向, 半周期内 `∫ omega dt = 0`,
//! 累计转角只在有限范围内往复 —— 表现为**原地小幅摇摆**, 而不是单调自转。
//! 角度极值出现在 `omega` 穿过 0 的那一刻, 大小为 `端点² / (2 * alpha_max)`;
//! (当前默认 `[-4, 4] / t_acc = 0.50` → 摆幅 `±4²/(2*16) = ±0.5 rad ≈ ±28.6°`。)
//!
//! 设计要点:
//! - 曲线推进与 ECS 完全解耦 (纯函数 [`advance_spin`]), 便于单元测试与数值验证。
//! - 时间/角度/角速度内部一律 `f64`, 只在构造 Bevy 四元数时降到 `f32`。
//! - 累计转角连续不重置; 每个实体持有独立状态与随机数种子。
//! - 使用仿真时间推进 (暂停即停止)。
//!
//! 单位: 角速度 rad/s, 角度 rad, 时间 s。

use bevy::prelude::*;

/// 规律变速模式的固定保持时间 (s)
const REGULAR_HOLD: f64 = 0.35;

/// 阶段切换的安全上限, 防止极端 dt 下死循环
const MAX_STAGES_PER_STEP: u32 = 256;

/// 三角波模式允许的最短加速时间 (s), 防止 `alpha_max` 大到数值上不可用
const MIN_T_ACC: f64 = 1e-3;

/// 旋转模式
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpinMode {
    /// 慢速固定旋转: 原地以 `slow_omega` (默认 3.0 rad/s) 匀速自转, 无加速段。
    /// 等价于改造前的 `Quat::from_rotation_y(3.0 * t)`。
    Slow,
    /// 三角波: `omega` 在 `[min_omega, max_omega]` 之间连续线性往复,
    /// `alpha` 恒为 ±`alpha_max` (纯方波, **没有匀速段**), `alpha_max = (max-min)/t_acc`。
    Triangle,
    /// 恒速基线: 加速到 `max_omega` 后恒定不变
    Constant,
    /// 规律变速: `min_omega` <-> `max_omega` 交替, 固定保持 0.35 s
    Regular,
    /// 不规则变速: [`min_omega`, `max_omega`] 内随机, 保持 `hold_min`~`hold_max`
    Irregular,
}

impl SpinMode {
    /// 供真值输出使用的数值编码
    pub fn as_u8(self) -> u8 {
        match self {
            SpinMode::Constant => 0,
            SpinMode::Regular => 1,
            SpinMode::Irregular => 2,
            SpinMode::Slow => 3,
            SpinMode::Triangle => 4,
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "slow" | "legacy" | "fixed" | "static" => Some(SpinMode::Slow),
            "triangle" | "tri" | "triwave" | "wave" => Some(SpinMode::Triangle),
            "constant" | "const" | "baseline" => Some(SpinMode::Constant),
            "regular" => Some(SpinMode::Regular),
            "irregular" | "random" => Some(SpinMode::Irregular),
            _ => None,
        }
    }
}

/// 曲线配置 (Resource)。可用环境变量覆盖, 见 [`SpinConfig::from_env`]。
#[derive(Resource, Clone, Debug)]
pub struct SpinConfig {
    /// 正常循环角速度下限 (rad/s)
    pub min_omega: f64,
    /// 正常循环角速度上限 (rad/s)
    pub max_omega: f64,
    /// 角加速度大小 (rad/s², >0)。用于 Constant / Regular / Irregular 三种模式。
    /// [`SpinMode::Triangle`] **不使用**该值, 它由 `t_acc` 反推, 见 [`SpinConfig::alpha_max`]。
    pub alpha: f64,
    /// [`SpinMode::Triangle`] 的单程加速/减速时间 (s, >0)。
    /// `alpha_max = (max_omega - min_omega) / t_acc`, 三角波周期 = `2 * t_acc`。
    pub t_acc: f64,
    /// 相邻目标之间的最小速度差 (rad/s)
    pub min_delta: f64,
    /// 保持时间下限 (s)
    pub hold_min: f64,
    /// 保持时间上限 (s)
    pub hold_max: f64,
    /// [`SpinMode::Slow`] 模式下的恒定角速度 (rad/s)
    pub slow_omega: f64,
    /// 随机种子 (固定以保证可复现)
    pub seed: u64,
    /// 模式
    pub mode: SpinMode,
}

impl Default for SpinConfig {
    fn default() -> Self {
        Self {
            min_omega: -4.0,
            max_omega: 4.0,
            alpha: 120.0,
            t_acc: 0.50,
            min_delta: 3.0,
            hold_min: 0.20,
            hold_max: 0.50,
            slow_omega: 3.0,
            seed: 20261009,
            // 默认 = 三角波 (omega -4 <-> +4 线性往复, alpha 恒为 ±16 rad/s² 纯方波, 周期 1.0 s);
            // 注意 omega 会持续穿过 0 换向, 半周期内 ∫omega dt = 0, 所以累计转角只在
            // ±4²/(2*16) = ±0.5 rad (±28.6°) 内往复 —— 敌方车是**原地摇摆**, 不是连续自转。
            // 想换回随机变速用 SPIN_MODE=irregular, 回到 3 rad/s 匀速用 SPIN_MODE=slow。
            mode: SpinMode::Triangle,
        }
    }
}

impl SpinConfig {
    /// 从环境变量读取并校验。
    ///
    /// 支持: `SPIN_MODE`(triangle|slow|constant|regular|irregular)、`SPIN_SLOW_OMEGA`、
    /// `SPIN_MIN`、`SPIN_MAX`、`SPIN_T_ACC`、`SPIN_ALPHA`、`SPIN_MIN_DELTA`、
    /// `SPIN_HOLD_MIN`、`SPIN_HOLD_MAX`、`SPIN_SEED`。
    pub fn from_env() -> Self {
        let mut cfg = Self::default();

        if let Ok(v) = std::env::var("SPIN_MODE") {
            match SpinMode::parse(&v) {
                Some(m) => cfg.mode = m,
                None => warn!("[SPIN] 无法识别的 SPIN_MODE={v:?}, 使用默认 {:?}", cfg.mode),
            }
        }
        if let Some(v) = env_f64("SPIN_SLOW_OMEGA") {
            cfg.slow_omega = v;
        }
        if let Some(v) = env_f64("SPIN_MIN") {
            cfg.min_omega = v;
        }
        if let Some(v) = env_f64("SPIN_MAX") {
            cfg.max_omega = v;
        }
        if let Some(v) = env_f64("SPIN_T_ACC") {
            cfg.t_acc = v;
        }
        if let Some(v) = env_f64("SPIN_ALPHA") {
            cfg.alpha = v;
        }
        if let Some(v) = env_f64("SPIN_MIN_DELTA") {
            cfg.min_delta = v;
        }
        if let Some(v) = env_f64("SPIN_HOLD_MIN") {
            cfg.hold_min = v;
        }
        if let Some(v) = env_f64("SPIN_HOLD_MAX") {
            cfg.hold_max = v;
        }
        if let Ok(v) = std::env::var("SPIN_SEED") {
            match v.trim().parse::<u64>() {
                Ok(s) => cfg.seed = s,
                Err(_) => warn!("[SPIN] 无法解析 SPIN_SEED={v:?}, 使用默认 {}", cfg.seed),
            }
        }

        cfg.validate();
        info!(
            "[SPIN] mode={:?} slow_omega={} omega=[{},{}] t_acc={} alpha={} (triangle: alpha_max={}, period={}) hold=[{},{}] seed={}",
            cfg.mode,
            cfg.slow_omega,
            cfg.min_omega,
            cfg.max_omega,
            cfg.t_acc,
            cfg.alpha,
            cfg.alpha_max(),
            cfg.t_acc * 2.0,
            cfg.hold_min,
            cfg.hold_max,
            cfg.seed
        );
        cfg
    }

    /// 校验并修正非法参数。返回是否发生过修正。
    pub fn validate(&mut self) -> bool {
        let d = Self::default();
        let mut fixed = false;

        if !self.min_omega.is_finite()
            || !self.max_omega.is_finite()
            || self.min_omega >= self.max_omega
        {
            warn!(
                "[SPIN] 非法速度范围 [{}, {}], 回退默认 [{}, {}]",
                self.min_omega, self.max_omega, d.min_omega, d.max_omega
            );
            self.min_omega = d.min_omega;
            self.max_omega = d.max_omega;
            fixed = true;
        }
        if !self.alpha.is_finite() || self.alpha <= 0.0 {
            warn!("[SPIN] 非法角加速度 {}, 回退默认 {}", self.alpha, d.alpha);
            self.alpha = d.alpha;
            fixed = true;
        }
        if !self.t_acc.is_finite() || self.t_acc < MIN_T_ACC {
            warn!(
                "[SPIN] 非法加速时间 {} (需 >= {}), 回退默认 {}",
                self.t_acc, MIN_T_ACC, d.t_acc
            );
            self.t_acc = d.t_acc;
            fixed = true;
        }
        if !self.hold_min.is_finite()
            || !self.hold_max.is_finite()
            || self.hold_min <= 0.0
            || self.hold_min > self.hold_max
        {
            warn!(
                "[SPIN] 非法保持时间 [{}, {}], 回退默认 [{}, {}]",
                self.hold_min, self.hold_max, d.hold_min, d.hold_max
            );
            self.hold_min = d.hold_min;
            self.hold_max = d.hold_max;
            fixed = true;
        }
        if !self.min_delta.is_finite() || self.min_delta < 0.0 {
            warn!("[SPIN] 非法最小速度差 {}, 回退默认 {}", self.min_delta, d.min_delta);
            self.min_delta = d.min_delta;
            fixed = true;
        }
        // min_delta 超过整个速度区间时永远选不出合法目标, 收敛到区间一半
        let span = self.max_omega - self.min_omega;
        if self.min_delta > span {
            warn!(
                "[SPIN] min_delta {} 超过速度区间 {}, 收敛为 {}",
                self.min_delta,
                span,
                span * 0.5
            );
            self.min_delta = span * 0.5;
            fixed = true;
        }
        if !self.slow_omega.is_finite() || self.slow_omega <= 0.0 {
            warn!(
                "[SPIN] 非法慢速角速度 {}, 回退默认 {}",
                self.slow_omega, d.slow_omega
            );
            self.slow_omega = d.slow_omega;
            fixed = true;
        }

        fixed
    }

    /// 三角波模式的角加速度幅值: `alpha_max = (max_omega - min_omega) / t_acc`。
    ///
    /// 该模式下 `alpha` 只取 `+alpha_max` / `-alpha_max`, **没有 0 (匀速) 段**。
    pub fn alpha_max(&self) -> f64 {
        (self.max_omega - self.min_omega) / self.t_acc
    }

    /// 从 0 启动时的首个目标角速度
    pub fn initial_target(&self) -> f64 {
        match self.mode {
            SpinMode::Slow => self.slow_omega,
            // Triangle 先向上限加速 (初始角速度由 `SpinState::from_config` 设到下限)
            _ => self.max_omega,
        }
    }
}

fn env_f64(key: &str) -> Option<f64> {
    match std::env::var(key) {
        Ok(v) => match v.trim().parse::<f64>() {
            Ok(x) => Some(x),
            Err(_) => {
                warn!("[SPIN] 无法解析 {key}={v:?}, 忽略该项");
                None
            }
        },
        Err(_) => None,
    }
}

/// 曲线阶段
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpinPhase {
    /// 变速: 以 ±`alpha` 逼近目标角速度
    Accelerating,
    /// 保持: 角速度恒定
    Holding,
}

impl SpinPhase {
    /// 供真值输出使用的数值编码
    pub fn as_u8(self) -> u8 {
        match self {
            SpinPhase::Accelerating => 0,
            SpinPhase::Holding => 1,
        }
    }
}

/// 每个敌方实体独立的旋转状态。
#[derive(Component, Clone, Debug)]
pub struct SpinState {
    /// 累计转角 (rad), 连续不重置
    pub angle: f64,
    /// 当前角速度 (rad/s)
    pub omega: f64,
    /// 目标角速度 (rad/s); 保持阶段等于当前角速度
    pub target: f64,
    /// 当前阶段
    pub phase: SpinPhase,
    /// 保持阶段剩余时间 (s); 恒速模式为无穷大
    pub hold_remaining: f64,
    /// 随机数状态 (SplitMix64)
    pub rng: u64,
    /// 最近一次阶段切换的仿真时间 (s)
    pub event_t: f64,
    /// 最近一次积分使用的角加速度 (rad/s²), 仅诊断用
    pub alpha_now: f64,
}

impl SpinState {
    /// 新建: 角速度 0, 阶段 = 变速, 首个目标为 `first_target`。
    pub fn new(seed: u64, first_target: f64) -> Self {
        Self {
            angle: 0.0,
            omega: 0.0,
            target: first_target,
            phase: SpinPhase::Accelerating,
            hold_remaining: 0.0,
            rng: seed,
            event_t: 0.0,
            alpha_now: 0.0,
        }
    }

    /// 按配置初始化。
    ///
    /// 与 [`SpinState::new`] 的区别: [`SpinMode::Triangle`] 从 `min_omega` 出发 (而不是 0),
    /// 这样第一帧起就是三角波本身, 不会出现 `0 -> min_omega` 的启动斜坡。
    pub fn from_config(cfg: &SpinConfig) -> Self {
        let mut st = Self::new(cfg.seed, cfg.initial_target());
        if cfg.mode == SpinMode::Triangle {
            st.omega = cfg.min_omega;
            st.target = cfg.max_omega;
        }
        st
    }
}

/// 推进曲线 `dt` 秒。
///
/// 单段内使用解析积分:
/// ```text
/// omega_next = omega + alpha * dt
/// angle_next = angle + omega * dt + 0.5 * alpha * dt * dt
/// ```
/// 若一帧跨越阶段边界, 先精确积分到边界, 再用剩余时间进入下一阶段。
pub fn advance_spin(state: &mut SpinState, cfg: &SpinConfig, dt: f64, sim_t: f64) {
    if !dt.is_finite() || dt <= 0.0 {
        return;
    }

    // 慢速固定旋转: 无加速段, 直接从第一帧起以 slow_omega 匀速累计。
    // 等价于改造前的 `angle = 3.0 * t`。
    if cfg.mode == SpinMode::Slow {
        state.omega = cfg.slow_omega;
        state.target = cfg.slow_omega;
        state.phase = SpinPhase::Holding;
        state.hold_remaining = f64::INFINITY;
        state.alpha_now = 0.0;
        state.angle += state.omega * dt;
        return;
    }

    // 三角波: omega 在 [min, max] 之间线性往复, alpha 恒为 ±alpha_max (纯方波, 无匀速段)。
    // 单段内仍用解析积分, 到端点时精确切段并立刻反号, 因此不会累积误差或越界。
    if cfg.mode == SpinMode::Triangle {
        let lo = cfg.min_omega;
        let hi = cfg.max_omega;
        let alpha_max = cfg.alpha_max();

        // 数值安全: 首次调用时 omega = 0, 直接钳到下限; 任何异常值也一并钳回区间。
        if !state.omega.is_finite() {
            state.omega = lo;
        }
        state.omega = state.omega.clamp(lo, hi);
        // 目标必须指向某一端点, 且不能正好是当前所在端点 (否则会原地"换向"卡住)
        if (state.target != lo && state.target != hi)
            || (state.omega - state.target).abs() <= 1e-9
        {
            state.target = if state.omega - lo <= hi - state.omega {
                hi
            } else {
                lo
            };
        }

        state.phase = SpinPhase::Accelerating; // 三角波没有保持段
        state.hold_remaining = 0.0;

        let mut remaining = dt;
        let mut guard = 0u32;
        while remaining > 1e-12 && guard < MAX_STAGES_PER_STEP {
            guard += 1;
            let diff = state.target - state.omega;
            if diff.abs() <= 1e-9 {
                // 已在端点: 立即换向, 不等待
                state.omega = state.target;
                state.target = if state.target >= hi - 1e-9 { lo } else { hi };
                state.event_t = sim_t - remaining;
                continue;
            }
            let alpha = alpha_max * diff.signum();
            let t_needed = (diff / alpha).abs();
            if remaining < t_needed {
                // 本步到不了端点
                state.angle += state.omega * remaining + 0.5 * alpha * remaining * remaining;
                state.omega += alpha * remaining;
                remaining = 0.0;
            } else {
                // 精确积分到端点, 剩余时间进入反向段
                state.angle += state.omega * t_needed + 0.5 * alpha * t_needed * t_needed;
                state.omega = state.target;
                remaining -= t_needed;
                state.target = if state.target >= hi - 1e-9 { lo } else { hi };
                state.event_t = sim_t - remaining;
            }
        }

        // 收尾: 钳位 + 按"接下来要走的方向"给出方波 alpha
        state.omega = state.omega.clamp(lo, hi);
        let diff = state.target - state.omega;
        state.alpha_now = if diff.abs() <= 1e-9 {
            if state.omega >= hi - 1e-9 {
                -alpha_max
            } else {
                alpha_max
            }
        } else {
            alpha_max * diff.signum()
        };
        return;
    }

    let mut remaining = dt;
    let mut guard = 0u32;

    while remaining > 1e-12 && guard < MAX_STAGES_PER_STEP {
        guard += 1;
        match state.phase {
            SpinPhase::Accelerating => {
                let diff = state.target - state.omega;
                if diff.abs() <= 1e-9 {
                    // 已在目标上, 直接进入保持
                    state.omega = state.target;
                    state.alpha_now = 0.0;
                    enter_hold(state, cfg);
                    state.event_t = sim_t - remaining;
                    continue;
                }

                let alpha = cfg.alpha * diff.signum();
                let t_needed = (diff / alpha).abs();

                if remaining < t_needed {
                    // 本步不足以到达目标
                    state.angle += state.omega * remaining + 0.5 * alpha * remaining * remaining;
                    state.omega += alpha * remaining;
                    state.alpha_now = alpha;
                    remaining = 0.0;
                } else {
                    // 精确积分到目标, 剩余时间进入保持
                    state.angle += state.omega * t_needed + 0.5 * alpha * t_needed * t_needed;
                    state.omega = state.target;
                    state.alpha_now = alpha;
                    remaining -= t_needed;
                    enter_hold(state, cfg);
                    state.event_t = sim_t - remaining;
                }
            }
            SpinPhase::Holding => {
                state.alpha_now = 0.0;
                if remaining < state.hold_remaining {
                    state.angle += state.omega * remaining;
                    state.hold_remaining -= remaining;
                    remaining = 0.0;
                } else {
                    // 保持结束: 积分到边界, 选下一个目标并回到变速
                    state.angle += state.omega * state.hold_remaining;
                    remaining -= state.hold_remaining;
                    state.hold_remaining = 0.0;
                    state.target = pick_target(state, cfg);
                    state.phase = SpinPhase::Accelerating;
                    state.event_t = sim_t - remaining;
                }
            }
        }
    }
}

/// 到达目标速度 -> 进入保持阶段, 抽取保持时长
fn enter_hold(state: &mut SpinState, cfg: &SpinConfig) {
    state.phase = SpinPhase::Holding;
    state.hold_remaining = match cfg.mode {
        // Slow 在 `advance_spin` 里提前返回, 不会走到这里; 保险起见与恒速一致。
        SpinMode::Slow | SpinMode::Constant => f64::INFINITY,
        // 三角波没有保持段 (advance_spin 里已提前返回), 保持时长 0 表示"立刻换向"。
        SpinMode::Triangle => 0.0,
        SpinMode::Regular => REGULAR_HOLD,
        SpinMode::Irregular => {
            cfg.hold_min + next_f64(&mut state.rng) * (cfg.hold_max - cfg.hold_min)
        }
    };
}

/// 抽取下一个目标角速度。
///
/// 不规则模式下保证与当前速度相差至少 `min_delta`:
/// 合法区间为 `[min, omega-min_delta] ∪ [omega+min_delta, max]`,
/// 直接构造区间后按长度加权抽样, 不使用无上限的重试循环。
fn pick_target(state: &mut SpinState, cfg: &SpinConfig) -> f64 {
    match cfg.mode {
        // Slow 在 `advance_spin` 里提前返回, 不会走到这里; 保险起见返回慢速值。
        SpinMode::Slow => cfg.slow_omega,
        // 三角波: 折返到另一端点 (advance_spin 里提前返回, 正常不会走到这里)
        SpinMode::Triangle => {
            if state.omega >= (cfg.min_omega + cfg.max_omega) * 0.5 {
                cfg.min_omega
            } else {
                cfg.max_omega
            }
        }
        SpinMode::Constant => cfg.max_omega,
        SpinMode::Regular => {
            let mid = (cfg.min_omega + cfg.max_omega) * 0.5;
            if state.omega >= mid {
                cfg.min_omega
            } else {
                cfg.max_omega
            }
        }
        SpinMode::Irregular => {
            let lo1 = cfg.min_omega;
            let hi1 = state.omega - cfg.min_delta;
            let lo2 = state.omega + cfg.min_delta;
            let hi2 = cfg.max_omega;

            let len1 = (hi1 - lo1).max(0.0);
            let len2 = (hi2 - lo2).max(0.0);
            let total = len1 + len2;

            if total <= 1e-12 {
                // 退化 (区间过窄): 全区间均匀兜底
                cfg.min_omega + next_f64(&mut state.rng) * (cfg.max_omega - cfg.min_omega)
            } else {
                let r = next_f64(&mut state.rng) * total;
                if r < len1 {
                    lo1 + r
                } else {
                    lo2 + (r - len1)
                }
            }
        }
    }
}

/// SplitMix64: 状态推进并返回下一个 64 位随机数
#[inline]
fn next_u64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// 返回 `[0, 1)` 的均匀随机数
#[inline]
fn next_f64(state: &mut u64) -> f64 {
    (next_u64(state) >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(cfg: &SpinConfig, dt: f64, total: f64) -> SpinState {
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        while t < total - 1e-12 {
            advance_spin(&mut st, cfg, dt, t);
            t += dt;
        }
        st
    }

    /// 变速相关测试的基准配置 (不规则变速)。
    /// [`SpinConfig::default`] 现在是 Triangle, 这里显式写出以表明测试意图。
    fn irregular() -> SpinConfig {
        SpinConfig {
            mode: SpinMode::Irregular,
            ..Default::default()
        }
    }

    /// 慢速模式: 恒定角速度、无加速段、累计角 = omega * t (等价改造前的 3.0*t)
    #[test]
    fn slow_mode_is_constant_omega() {
        let cfg = SpinConfig {
            mode: SpinMode::Slow,
            ..Default::default()
        };
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let dt = 1.0 / 60.0;
        let mut t = 0.0;
        for _ in 0..600 {
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            assert!(
                (st.omega - cfg.slow_omega).abs() < 1e-12,
                "omega={} 应恒为 {}",
                st.omega,
                cfg.slow_omega
            );
            assert_eq!(st.phase, SpinPhase::Holding);
            assert_eq!(st.alpha_now, 0.0);
        }
        // 10 s 后累计角 ≈ 3.0 * 10
        assert!(
            (st.angle - cfg.slow_omega * 10.0).abs() < 1e-6,
            "angle={}",
            st.angle
        );
    }

    /// 恒速模式最终应稳定在 max_omega
    #[test]
    fn constant_reaches_max() {
        let cfg = SpinConfig {
            mode: SpinMode::Constant,
            ..Default::default()
        };
        let st = run(&cfg, 1.0 / 60.0, 5.0);
        assert!((st.omega - cfg.max_omega).abs() < 1e-6, "omega={}", st.omega);
    }

    /// 正常循环内角速度不越界 [min, max]
    #[test]
    fn omega_within_range() {
        let cfg = irregular();
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        // 跳过启动段
        for _ in 0..60 {
            advance_spin(&mut st, &cfg, 1.0 / 60.0, t);
            t += 1.0 / 60.0;
        }
        for _ in 0..6000 {
            advance_spin(&mut st, &cfg, 1.0 / 60.0, t);
            t += 1.0 / 60.0;
            assert!(
                st.omega >= cfg.min_omega - 1e-9 && st.omega <= cfg.max_omega + 1e-9,
                "omega={} 越界",
                st.omega
            );
        }
    }

    /// 到达目标时无超调: 角速度不应越过目标再回来
    #[test]
    fn no_overshoot() {
        let cfg = irregular();
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        let mut prev = st.omega;
        let mut prev_target = st.target;
        for _ in 0..6000 {
            advance_spin(&mut st, &cfg, 1.0 / 60.0, t);
            t += 1.0 / 60.0;
            if st.target != prev_target {
                // 目标切换时允许方向改变
                prev_target = st.target;
                prev = st.omega;
                continue;
            }
            // 同一目标内, 角速度应单调趋近, 不越过
            if (prev_target - prev).abs() > 1e-9 {
                let dir = (prev_target - prev).signum();
                assert!(
                    dir * (st.omega - prev) >= -1e-9,
                    "出现反向/超调: prev={} now={} target={}",
                    prev,
                    st.omega,
                    prev_target
                );
            }
            prev = st.omega;
        }
    }

    /// 变速段斜率应为 ±alpha, 保持段为 0
    #[test]
    fn slope_matches_alpha() {
        let cfg = irregular();
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let dt = 1.0 / 1000.0;
        let mut t = 0.0;
        for _ in 0..3000 {
            let before = (st.omega, st.phase, st.target);
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            if st.phase == SpinPhase::Accelerating && before.1 == SpinPhase::Accelerating {
                let d = (st.omega - before.0) / dt;
                if d.abs() > 1e-9 {
                    assert!(
                        (d.abs() - cfg.alpha).abs() < 1e-6,
                        "斜率 {} 不等于 alpha {}",
                        d,
                        cfg.alpha
                    );
                }
            }
        }
    }

    /// 大 dt 与小步推进结果一致 (跨阶段正确性)
    #[test]
    fn big_step_matches_small_steps() {
        let cfg = irregular();

        let mut a = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        while t < 3.0 - 1e-12 {
            advance_spin(&mut a, &cfg, 0.1, t);
            t += 0.1;
        }

        let mut b = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        while t < 3.0 - 1e-12 {
            advance_spin(&mut b, &cfg, 1.0 / 600.0, t);
            t += 1.0 / 600.0;
        }

        // 用相对误差比较累计角度
        let denom = a.angle.abs().max(1.0);
        let rel = (a.angle - b.angle).abs() / denom;
        assert!(rel < 1e-3, "大/小步累计角度不一致: {} vs {}", a.angle, b.angle);
    }

    /// 相同种子 -> 可复现
    #[test]
    fn deterministic_with_seed() {
        let cfg = irregular();
        let a = run(&cfg, 1.0 / 60.0, 20.0);
        let b = run(&cfg, 1.0 / 60.0, 20.0);
        assert_eq!(a.omega.to_bits(), b.omega.to_bits());
        assert_eq!(a.target.to_bits(), b.target.to_bits());
    }

    /// 不同种子 -> 序列不同
    #[test]
    fn different_seeds_differ() {
        let cfg = irregular();
        let a = run(&cfg, 1.0 / 60.0, 20.0);
        let cfg2 = SpinConfig {
            seed: cfg.seed + 1,
            ..cfg.clone()
        };
        let b = run(&cfg2, 1.0 / 60.0, 20.0);
        assert!(a.angle.to_bits() != b.angle.to_bits());
    }

    /// 非法配置会被修正, 不 panic、不死循环
    #[test]
    fn invalid_config_is_fixed() {
        let mut cfg = SpinConfig {
            min_omega: 20.0,
            max_omega: 10.0,
            alpha: -5.0,
            hold_min: 0.5,
            hold_max: 0.1,
            min_delta: 999.0,
            ..Default::default()
        };
        assert!(cfg.validate());
        assert!(cfg.min_omega < cfg.max_omega);
        assert!(cfg.alpha > 0.0);
        assert!(cfg.hold_min <= cfg.hold_max);
        assert!(cfg.min_delta <= cfg.max_omega - cfg.min_omega);
        // 仍可正常运行
        let _ = run(&cfg, 1.0 / 60.0, 2.0);
    }

    /// 规律模式应在 min/max 之间交替
    #[test]
    fn regular_alternates() {
        let cfg = SpinConfig {
            mode: SpinMode::Regular,
            ..Default::default()
        };
        let mut st = SpinState::new(cfg.seed, cfg.initial_target());
        let mut t = 0.0;
        let mut targets = Vec::new();
        let mut last = st.target;
        for _ in 0..6000 {
            advance_spin(&mut st, &cfg, 1.0 / 60.0, t);
            t += 1.0 / 60.0;
            if st.target != last {
                targets.push(st.target);
                last = st.target;
            }
        }
        assert!(targets.len() >= 4, "切换次数太少: {targets:?}");
        for w in targets.windows(2) {
            assert!((w[0] - w[1]).abs() > 1.0, "未在 min/max 间交替: {targets:?}");
        }
    }

    // ================= 三角波模式 =================

    /// 三角波基准配置: 10 ~ 18 rad/s, 单程 0.20 s -> alpha_max = 40 rad/s²
    fn triangle() -> SpinConfig {
        SpinConfig {
            mode: SpinMode::Triangle,
            min_omega: 10.0,
            max_omega: 18.0,
            t_acc: 0.20,
            ..Default::default()
        }
    }

    /// alpha_max 必须等于 (max - min) / t_acc
    #[test]
    fn triangle_alpha_max_from_t_acc() {
        let cfg = triangle();
        assert!((cfg.alpha_max() - 40.0).abs() < 1e-12, "{}", cfg.alpha_max());
        let cfg2 = SpinConfig { t_acc: 0.5, ..triangle() };
        assert!((cfg2.alpha_max() - 16.0).abs() < 1e-12, "{}", cfg2.alpha_max());
        // 非法 t_acc 会被 validate 修正
        let mut bad = SpinConfig { t_acc: 0.0, ..triangle() };
        assert!(bad.validate());
        assert!(bad.t_acc > 0.0);
    }

    /// alpha 必须是纯方波: 只取 ±alpha_max, 绝不为 0, 且始终处于变速阶段
    #[test]
    fn triangle_alpha_is_pure_square_wave() {
        let cfg = triangle();
        let am = cfg.alpha_max();
        let mut st = SpinState::from_config(&cfg);
        let dt = 1.0 / 1000.0;
        let mut t = 0.0;
        let (mut saw_pos, mut saw_neg) = (false, false);
        for _ in 0..20_000 {
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            assert!(st.alpha_now != 0.0, "alpha 出现 0 (匀速段)");
            assert!(
                (st.alpha_now.abs() - am).abs() < 1e-9,
                "alpha={} 不是 ±{}",
                st.alpha_now,
                am
            );
            assert_eq!(st.phase, SpinPhase::Accelerating, "三角波不应出现保持段");
            if st.alpha_now > 0.0 {
                saw_pos = true;
            } else {
                saw_neg = true;
            }
        }
        assert!(saw_pos && saw_neg, "alpha 未正负交替");
    }

    /// omega 始终在 [min, max] 内, 且两端都要到达
    #[test]
    fn triangle_omega_stays_in_range_and_hits_both_ends() {
        let cfg = triangle();
        let mut st = SpinState::from_config(&cfg);
        let dt = 1.0 / 1000.0;
        let mut t = 0.0;
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        for _ in 0..20_000 {
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            assert!(
                st.omega >= cfg.min_omega - 1e-9 && st.omega <= cfg.max_omega + 1e-9,
                "omega={} 越界",
                st.omega
            );
            lo = lo.min(st.omega);
            hi = hi.max(st.omega);
        }
        assert!((lo - cfg.min_omega).abs() < 1e-6, "未到达下限 min={lo}");
        assert!((hi - cfg.max_omega).abs() < 1e-6, "未到达上限 max={hi}");
    }

    /// 单程时间 (相邻两次换向的间隔) 必须等于 t_acc
    #[test]
    fn triangle_ramp_time_matches_t_acc() {
        let cfg = triangle();
        let mut st = SpinState::from_config(&cfg);
        let dt = 1.0 / 10_000.0;
        let mut t = 0.0;
        let mut flips: Vec<f64> = Vec::new();
        let mut last = st.target;
        for _ in 0..200_000 {
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            if st.target != last {
                flips.push(t);
                last = st.target;
            }
        }
        assert!(flips.len() >= 8, "换向次数太少: {}", flips.len());
        for w in flips.windows(2) {
            let d = w[1] - w[0];
            assert!(
                (d - cfg.t_acc).abs() < 1e-3,
                "单程时间 {} != t_acc {}",
                d,
                cfg.t_acc
            );
        }
    }

    /// omega 必须是 alpha 的积分: d(omega)/dt == alpha_now (换向那一步除外)
    #[test]
    fn triangle_omega_is_integral_of_alpha() {
        let cfg = triangle();
        let mut st = SpinState::from_config(&cfg);
        let dt = 1.0 / 2000.0;
        let mut t = 0.0;
        let mut checked = 0;
        for _ in 0..40_000 {
            let before = st.omega;
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            let slope = (st.omega - before) / dt;
            if (slope - st.alpha_now).abs() > 1e-6 {
                continue; // 这一步跨了端点, alpha 已反号
            }
            assert!((slope - st.alpha_now).abs() < 1e-6, "d(omega)/dt != alpha");
            checked += 1;
        }
        assert!(checked > 30_000, "有效步数太少: {checked}");
    }

    /// 大 dt 与小步推进结果一致 (跨端点正确性)
    #[test]
    fn triangle_big_step_matches_small_steps() {
        let cfg = triangle();
        let mut a = SpinState::from_config(&cfg);
        let mut t = 0.0;
        while t < 5.0 - 1e-12 {
            advance_spin(&mut a, &cfg, 0.1, t);
            t += 0.1;
        }
        let mut b = SpinState::from_config(&cfg);
        let mut t = 0.0;
        while t < 5.0 - 1e-12 {
            advance_spin(&mut b, &cfg, 1.0 / 2000.0, t);
            t += 1.0 / 2000.0;
        }
        let rel = (a.angle - b.angle).abs() / b.angle.abs().max(1.0);
        assert!(rel < 1e-4, "大/小步累计角不一致: {} vs {}", a.angle, b.angle);
        assert!((a.omega - b.omega).abs() < 1e-3, "omega 不一致");
    }

    // ================= 负下限三角波 (原地摇摆) =================

    /// 负下限基准配置: -4 ~ +4 rad/s, 单程 0.50 s -> alpha_max = 16 rad/s² (与当前默认一致)。
    /// omega 会持续穿过 0 换向, 半周期 `∫ omega dt = 0`, 累计转角只在有限范围内往复。
    fn triangle_negative() -> SpinConfig {
        SpinConfig {
            mode: SpinMode::Triangle,
            min_omega: -4.0,
            max_omega: 4.0,
            t_acc: 0.50,
            ..Default::default()
        }
    }

    /// 负下限: alpha_max = (max - min) / t_acc = 8 / 0.50 = 16, 且默认配置本身合法
    #[test]
    fn triangle_negative_alpha_max() {
        let mut cfg = triangle_negative();
        assert!((cfg.alpha_max() - 16.0).abs() < 1e-12, "{}", cfg.alpha_max());
        assert!(!cfg.validate(), "负下限配置是合法的, 不该被 validate 修正");
    }

    /// 出厂默认 `SpinConfig::default()` 必须与文档一致: -4 ~ +4 / t_acc = 0.50
    #[test]
    fn default_config_is_minus4_to_4_period_1s() {
        let cfg = SpinConfig::default();
        assert_eq!(cfg.mode, SpinMode::Triangle);
        assert!((cfg.min_omega - (-4.0)).abs() < 1e-12);
        assert!((cfg.max_omega - 4.0).abs() < 1e-12);
        assert!((cfg.t_acc - 0.50).abs() < 1e-12);
        assert!((cfg.alpha_max() - 16.0).abs() < 1e-12);
        assert!((2.0 * cfg.t_acc - 1.0).abs() < 1e-12, "周期应为 1 s");
    }

    /// 负下限: omega 仍精确扫过 [-4, +4], alpha 恒为 ±16, 且摆幅等于解析极值
    #[test]
    fn triangle_negative_wobbles_within_t_acc() {
        let cfg = triangle_negative();
        let am = cfg.alpha_max();
        // 角度极值出现在 omega 穿过 0 的那一刻 (t = |端点| / alpha_max),
        // 该处角度 = ∓(端点)² / (2 * alpha_max)。整个摆动范围就是这两个极值。
        let amp = cfg.min_omega.abs().powi(2).max(cfg.max_omega.abs().powi(2)) / (2.0 * am);
        let mut st = SpinState::from_config(&cfg);
        let dt = 1.0 / 500.0;
        let mut t = 0.0;
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut a_lo, mut a_hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for _ in 0..5000 {
            advance_spin(&mut st, &cfg, dt, t);
            t += dt;
            assert!(
                st.omega >= cfg.min_omega - 1e-9 && st.omega <= cfg.max_omega + 1e-9,
                "omega={} 越界",
                st.omega
            );
            assert!(
                (st.alpha_now.abs() - am).abs() < 1e-9,
                "alpha={} 应为 ±{}",
                st.alpha_now,
                am
            );
            lo = lo.min(st.omega);
            hi = hi.max(st.omega);
            a_lo = a_lo.min(st.angle);
            a_hi = a_hi.max(st.angle);
        }
        assert!((lo - cfg.min_omega).abs() < 1e-6, "未到达下限 min={lo}");
        assert!((hi - cfg.max_omega).abs() < 1e-6, "未到达上限 max={hi}");
        // 摆动幅度 = 解析极值 (半个帧步的采样误差内)
        assert!(
            a_hi - amp < 1e-3 && a_lo + amp > -1e-3,
            "摆动范围 [{}, {}] 应逼近 ±{}",
            a_lo,
            a_hi,
            amp
        );
        assert!(
            (a_hi - a_lo) > amp * 1.9,
            "摆动峰峰值 {} 远小于 2*amp {}, 曲线可能没走满",
            a_hi - a_lo,
            2.0 * amp
        );
    }
}
