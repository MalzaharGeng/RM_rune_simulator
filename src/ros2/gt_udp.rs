//! 真值 UDP 发送模块
//!
//! 把仿真里的真值 (云台角 / 敌方装甲板位置) 通过 UDP 以 JSON 格式发给 Linux 小主机上的
//! PlotJuggler "UDP Server" 插件 (默认端口 9870)。
//!
//! 协议格式 (PlotJuggler ParserJSON):
//! ```json
//! {
//!   "timestamp": <秒, 浮点>,
//!   "gt": {
//!     "gimbal_yaw": 0.12,
//!     "gimbal_pitch": 0.05,
//!     "enemy_center_x": -4.0, "enemy_center_y": -7.0, "enemy_center_z": 0.195,
//!     "enemy_armor_1_x": 1.0, "enemy_armor_1_y": 2.0, "enemy_armor_1_z": 0.3,
//!     "spin_angle": 3.14, "spin_omega": 15.2, "spin_target": 18.0,
//!     "spin_alpha": 120.0, "spin_phase": 0, "spin_mode": 2, "spin_event_t": 1.83,
//!     "spin_t_acc": 0.2,
//!     ...
//!   }
//! }
//! ```
//! PlotJuggler 会把嵌套字典展平为 `gt/gimbal_yaw` 这样的曲线名。
//!
//! 目标地址/端口通过环境变量控制 (不设置则用默认值):
//!   GT_UDP_TARGET=192.168.x.x:9870
//!   GT_UDP_PORT=9870            (只改端口时用)
//!   GT_UDP_ENABLE=0             (设为 0 可整体关闭真值 UDP 发送)

use bevy::prelude::*;
use std::fmt::Write;
use std::net::UdpSocket;

/// 默认端口: PlotJuggler UDP Server 的默认监听端口
const DEFAULT_PORT: u16 = 9870;

/// 敌方步兵「变速小陀螺」真值快照 (用于曲线对齐与自瞄验证)。
///
/// 角度/角速度均为 ROS 系语义: `angle` 即 yaw (绕 +Z, rad), 与 Bevy 绕 +Y 转角同号。
#[derive(Clone, Copy, Debug, Default)]
pub struct SpinGt {
    /// 旋转模式编码: 0=恒定, 1=规律变速, 2=不规则变速, 3=慢速匀速, 4=三角波
    pub mode: u8,
    /// 累计转角 (rad, 即 ROS yaw)
    pub angle: f64,
    /// 当前角速度 (rad/s)
    pub omega: f64,
    /// 当前积分所用角加速度 (rad/s², 保持段为 0)
    pub alpha: f64,
    /// 目标角速度 (rad/s, 保持段等于当前角速度)
    pub target: f64,
    /// 阶段编码: 0=变速, 1=保持
    pub phase: u8,
    /// 最近一次阶段切换的仿真时间 (s)
    pub event_t: f64,
    /// 三角波模式的单程加速时间 (s); 其余模式为配置值, 不参与曲线
    pub t_acc: f64,
}

/// 真值 UDP 发送器
pub struct GtUdpSender {
    socket: Option<UdpSocket>,
    target: String,
    enabled: bool,
}

impl GtUdpSender {
    /// 从环境变量读取配置并创建发送器
    pub fn from_env() -> Self {
        let enabled = std::env::var("GT_UDP_ENABLE")
            .map(|v| v != "0" && v.to_lowercase() != "false")
            .unwrap_or(true);

        let target = std::env::var("GT_UDP_TARGET")
            .or_else(|_| {
                std::env::var("GT_UDP_PORT").map(|p| format!("127.0.0.1:{p}"))
            })
            .unwrap_or_else(|_| format!("127.0.0.1:{DEFAULT_PORT}"));

        if !enabled {
            info!("[GT-UDP] 已通过 GT_UDP_ENABLE 关闭真值 UDP 发送");
            return Self {
                socket: None,
                target,
                enabled: false,
            };
        }

        match UdpSocket::bind("0.0.0.0:0") {
            Ok(socket) => {
                // 允许向广播地址发送 (跨机时若用广播地址需要)
                let _ = socket.set_broadcast(true);
                info!("[GT-UDP] 真值 UDP 已启用, 目标 = {target}");
                Self {
                    socket: Some(socket),
                    target,
                    enabled: true,
                }
            }
            Err(err) => {
                error!("[GT-UDP] 创建 UDP socket 失败: {err:?}; 真值 UDP 已禁用");
                Self {
                    socket: None,
                    target,
                    enabled: false,
                }
            }
        }
    }

    /// 是否处于启用状态
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// 发送一帧真值。
    ///
    /// - `timestamp`: 仿真时间 (秒)
    /// - `enemy_center`: 敌方步兵车体中心 (ROS 系); 无敌方实体时为 None
    /// - `armors`: 4 块敌方装甲板位置, 顺序固定; 缺位用 None (JSON 里该字段省略)
    /// - `spin`: 敌方步兵变速小陀螺状态; 无敌方实体时为 None (相关字段省略)
    pub fn send(
        &self,
        timestamp: f64,
        gimbal_yaw: f64,
        gimbal_pitch: f64,
        gimbal_pos: [f64; 3],
        enemy_center: Option<[f64; 3]>,
        armors: &[Option<[f64; 3]>; 4],
        spin: Option<SpinGt>,
    ) {
        let Some(socket) = self.socket.as_ref() else {
            return;
        };

        let payload = build_json(
            timestamp,
            gimbal_yaw,
            gimbal_pitch,
            gimbal_pos,
            enemy_center,
            armors,
            spin,
        );
        if let Err(err) = socket.send_to(payload.as_bytes(), &self.target) {
            // 只在第一次失败时吵闹, 避免刷屏 —— 这里简单 warn 一次也可接受
            warn!("[GT-UDP] 发送失败 (target={}): {err:?}", self.target);
        }
    }
}

/// 手工拼 JSON, 避免引入 serde 序列化开销/依赖。
/// 数值格式化成足够精度, 保证曲线平滑。
fn build_json(
    timestamp: f64,
    gimbal_yaw: f64,
    gimbal_pitch: f64,
    gimbal_pos: [f64; 3],
    enemy_center: Option<[f64; 3]>,
    armors: &[Option<[f64; 3]>; 4],
    spin: Option<SpinGt>,
) -> String {
    // 预分配一个够用的容量, 减少 realloc
    let mut s = String::with_capacity(900);
    s.push_str("{\"timestamp\":");
    push_f64(&mut s, timestamp);
    s.push_str(",\"gt\":{");

    // 云台角
    s.push_str("\"gimbal_yaw\":");
    push_f64(&mut s, gimbal_yaw);
    s.push_str(",\"gimbal_pitch\":");
    push_f64(&mut s, gimbal_pitch);

    // 云台位置
    s.push_str(",\"gimbal_x\":");
    push_f64(&mut s, gimbal_pos[0]);
    s.push_str(",\"gimbal_y\":");
    push_f64(&mut s, gimbal_pos[1]);
    s.push_str(",\"gimbal_z\":");
    push_f64(&mut s, gimbal_pos[2]);

    // 敌方步兵车体中心 (装甲板位置 = 车体中心 + Rz(spin_angle) · 局部偏移)
    if let Some(c) = enemy_center {
        s.push_str(",\"enemy_center_x\":");
        push_f64(&mut s, c[0]);
        s.push_str(",\"enemy_center_y\":");
        push_f64(&mut s, c[1]);
        s.push_str(",\"enemy_center_z\":");
        push_f64(&mut s, c[2]);
    }

    // 4 块装甲板: 展平成 armor_1_x / armor_1_y / armor_1_z ...
    for (i, armor) in armors.iter().enumerate() {
        let Some(p) = armor else { continue };
        let n = i + 1;
        s.push_str(",\"enemy_armor_");
        push_u32(&mut s, n as u32);
        s.push_str("_x\":");
        push_f64(&mut s, p[0]);
        s.push_str(",\"enemy_armor_");
        push_u32(&mut s, n as u32);
        s.push_str("_y\":");
        push_f64(&mut s, p[1]);
        s.push_str(",\"enemy_armor_");
        push_u32(&mut s, n as u32);
        s.push_str("_z\":");
        push_f64(&mut s, p[2]);
    }

    // 敌方步兵变速小陀螺真值 (曲线对齐用)
    if let Some(sp) = spin {
        s.push_str(",\"spin_mode\":");
        push_u32(&mut s, sp.mode as u32);
        s.push_str(",\"spin_angle\":");
        push_f64(&mut s, sp.angle);
        s.push_str(",\"spin_omega\":");
        push_f64(&mut s, sp.omega);
        s.push_str(",\"spin_alpha\":");
        push_f64(&mut s, sp.alpha);
        s.push_str(",\"spin_target\":");
        push_f64(&mut s, sp.target);
        s.push_str(",\"spin_phase\":");
        push_u32(&mut s, sp.phase as u32);
        s.push_str(",\"spin_event_t\":");
        push_f64(&mut s, sp.event_t);
        s.push_str(",\"spin_t_acc\":");
        push_f64(&mut s, sp.t_acc);
    }

    s.push_str("}}");
    s
}

/// 写 f64: NaN/Inf 写成 JSON null (PlotJuggler 会断线, 一眼可辨)
#[inline]
fn push_f64(s: &mut String, v: f64) {
    if v.is_finite() {
        // 用 {:?} 打印 f64, Rust 会给出最短往返表示, 精度足够
        write!(s, "{v:?}").expect("writing to a String cannot fail");
    } else {
        s.push_str("null");
    }
}

#[inline]
fn push_u32(s: &mut String, v: u32) {
    write!(s, "{v}").expect("writing to a String cannot fail");
}
