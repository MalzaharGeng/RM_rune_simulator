//! 逐发事件埋点（只读诊断，不影响任何物理逻辑）。
//!
//! 目的：把「自瞄 fire 请求 → 仿真实际离膛 → 命中哪块板」这条事件链
//! 逐发记录下来，用于与 GT 板位序列对齐，定量测量：
//!
//! * `req_to_launch_ms` —— 从"最近一次 fire 请求到达"到"实际离膛"的延迟。
//!   这是本模块存在的主要理由：`main.rs::projectile_launch` 里 cooldown
//!   是一个自由运行的 50 ms 定时器，只在它到点的那一帧才检查 fire 请求，
//!   所以离膛时刻相对请求时刻存在 [0, 50ms) 的量化抖动（期望 25 ms）。
//!   在 7.5 rad/s 小陀螺下 25 ms 折合 10.7°。
//! * 离膛时刻云台的实际 yaw/pitch —— 与自瞄 `plan.target_yaw/pitch` 对比。
//! * 命中板名 + 是否近侧 + 命中时刻 —— 与 GT 板位对齐后可算真实脱靶量。
//!
//! 输出：环境变量 `SHOT_LOG` 指定的路径，每行一个 JSON 对象（JSONL）。
//! 不设该变量时整个模块短路，默认构建行为与改动前完全一致。
//!
//! ⚠️ 只记录，不改变任何决策。没有"命中即销毁"，弹丸寿命不变。
//!
//! 时钟：内部自建单调时钟（进程内 `Instant` 基准），因为 fire 请求来自
//! ROS2 异步线程、离膛/命中来自 Bevy 主循环，两者必须共用同一时间轴。

use std::collections::HashMap;
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use bevy::prelude::*;

/// 弹丸实体上挂的自增编号，用来把离膛事件和命中事件关联起来。
#[derive(Component, Clone, Copy, Debug)]
pub struct ShotId(pub u32);

#[derive(Debug, Clone, Copy)]
struct PendingLaunch {
    req_to_launch_ms: f32,
}

#[derive(Debug)]
struct ShotLogState {
    epoch: Instant,
    file: Option<std::fs::File>,
    next_id: u32,
    /// 最近一次自瞄 fire 请求：到达时刻 + 该请求的 yaw/pitch。
    last_fire_req: Option<(f64, f32, f32)>,
    /// shot_id -> 离膛信息，命中/漏弹时取用。
    pending: HashMap<u32, PendingLaunch>,
}

/// 可跨线程克隆的句柄（ROS2 异步线程 + Bevy 主循环共用）。
#[derive(Resource, Clone, Default)]
pub struct ShotLog(pub Arc<Mutex<ShotLogState>>);

impl Default for ShotLogState {
    fn default() -> Self {
        Self {
            epoch: Instant::now(),
            file: None,
            next_id: 1,
            last_fire_req: None,
            pending: HashMap::new(),
        }
    }
}

impl ShotLog {
    /// 读 `SHOT_LOG` 环境变量并打开输出文件。未设置则整体短路。
    pub fn from_env() -> Self {
        let path = std::env::var("SHOT_LOG").ok().filter(|s| !s.trim().is_empty());
        let file = path.and_then(|p| match OpenOptions::new().create(true).append(true).open(&p) {
            Ok(f) => {
                bevy::log::info!("[SHOTLOG] per-shot events -> {p}");
                Some(f)
            }
            Err(e) => {
                bevy::log::warn!("[SHOTLOG] cannot open {p}: {e}; disabled");
                None
            }
        });
        Self(Arc::new(Mutex::new(ShotLogState {
            epoch: Instant::now(),
            file,
            ..Default::default()
        })))
    }

    fn enabled(&self) -> bool {
        self.0.lock().map(|s| s.file.is_some()).unwrap_or(false)
    }

    /// 自瞄 fire 请求到达（ROS2 线程调用）。epoch = 进程启动 Instant。
    pub fn on_fire_request(&self, yaw: f32, pitch: f32, distance: f32) {
        if !self.enabled() {
            return;
        }
        let mut st = match self.0.lock() {
            Ok(s) => s,
            Err(_) => return,
        };
        let t = st.epoch.elapsed().as_secs_f64();
        st.last_fire_req = Some((t, yaw, pitch));
        if let Some(f) = &mut st.file {
            let _ = writeln!(
                f,
                r#"{{"ev":"req","t":{t:.6},"yaw":{yaw:.6},"pitch":{pitch:.6},"dist":{distance:.4}}}"#
            );
        }
    }

    /// 实际离膛。返回分配给该发弹丸的 `ShotId`。
    pub fn on_launch(&self, dist: f32) -> ShotId {
        let mut st = match self.0.lock() {
            Ok(s) => s,
            Err(_) => return ShotId(0),
        };
        let id = st.next_id;
        st.next_id += 1;
        if st.file.is_none() {
            return ShotId(id);
        }
        let t = st.epoch.elapsed().as_secs_f64();
        let (req_ms, req_yaw, req_pitch) = match st.last_fire_req.take() {
            Some((rt, ry, rp)) => (((t - rt) * 1000.0) as f32, ry, rp),
            None => (f32::NAN, f32::NAN, f32::NAN),
        };
        st.pending.insert(id, PendingLaunch { req_to_launch_ms: req_ms });
        if let Some(f) = &mut st.file {
            let _ = writeln!(
                f,
                r#"{{"ev":"launch","t":{t:.6},"shot_id":{id},"req_to_launch_ms":{req_ms:.2},"req_yaw":{req_yaw:.6},"req_pitch":{req_pitch:.6},"dist":{dist:.4}}}"#
            );
        }
        ShotId(id)
    }

    /// 命中（含近侧标记）。
    pub fn on_hit(&self, id: u32, plate: &str, near: bool) {
        if !self.enabled() {
            return;
        }
        let mut st = match self.0.lock() {
            Ok(s) => s,
            Err(_) => return,
        };
        let t = st.epoch.elapsed().as_secs_f64();
        let req_ms = st
            .pending
            .remove(&id)
            .map(|p| p.req_to_launch_ms)
            .unwrap_or(f32::NAN);
        if let Some(f) = &mut st.file {
            let near_i = if near { 1 } else { 0 };
            let _ = writeln!(
                f,
                r#"{{"ev":"hit","t":{t:.6},"shot_id":{id},"plate":"{plate}","near":{near_i},"req_to_launch_ms":{req_ms:.2}}}"#
            );
        }
    }

    /// 弹丸寿命结束且从未命中（漏弹）。
    pub fn on_miss(&self, id: u32) {
        if !self.enabled() {
            return;
        }
        let mut st = match self.0.lock() {
            Ok(s) => s,
            Err(_) => return,
        };
        let t = st.epoch.elapsed().as_secs_f64();
        if st.pending.remove(&id).is_none() {
            return; // 已判过命中
        }
        if let Some(f) = &mut st.file {
            let _ = writeln!(f, r#"{{"ev":"miss","t":{t:.6},"shot_id":{id}}}"#);
        }
    }
}
