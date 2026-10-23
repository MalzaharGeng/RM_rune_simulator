static mut LAUNCH_COUNT: u32 = 0;
static mut ACCURATE_COUNT: u32 = 0;
static mut INFANTRY_HIT_COUNT: u32 = 0;
static mut INFANTRY_NEAR_HIT_COUNT: u32 = 0;

pub fn increase_launch() {
    unsafe {
        LAUNCH_COUNT += 1;
    }
}

pub fn launch_count() -> u32 {
    unsafe { LAUNCH_COUNT }
}

pub fn increase_accurate() {
    unsafe {
        ACCURATE_COUNT += 1;
    }
}

pub fn accurate_count() -> u32 {
    unsafe { ACCURATE_COUNT }
}

pub fn accurate_pct() -> f32 {
    let l = launch_count();
    if l == 0 {
        return 0.0;
    }
    (accurate_count() as f32) / (launch_count() as f32)
}

pub fn increase_infantry_hit() {
    unsafe {
        INFANTRY_HIT_COUNT += 1;
    }
}

pub fn infantry_hit_count() -> u32 {
    unsafe { INFANTRY_HIT_COUNT }
}

pub fn infantry_hit_pct() -> f32 {
    let launches = launch_count();
    if launches == 0 {
        return 0.0;
    }
    (infantry_hit_count() as f32) / (launches as f32)
}

/// 近侧命中：被击中的那块装甲板的外法线朝向炮口，也就是自瞄本来打算打的那一块。
///
/// 为什么单独记这个：`infantry_hit_pct` 被仿真几何放大了 —— 弹丸会穿过车体
/// (碰撞层不相交)，而 4 块板间隔 90°、切向角容差 ±24.7°，所以打错一块板、
/// 或者穿堂打到对面板，都照样算命中。yaw 误差完全随机时它仍有约 0.79，
/// 健康估计是 1.00，区分不出好坏。近侧命中把烂估计压到约 0.5，才有区分度。
/// 量化脚本见 tools/hit_prob_model.py。
pub fn increase_infantry_near_hit() {
    unsafe {
        INFANTRY_NEAR_HIT_COUNT += 1;
    }
}

pub fn infantry_near_hit_count() -> u32 {
    unsafe { INFANTRY_NEAR_HIT_COUNT }
}

pub fn infantry_near_hit_pct() -> f32 {
    let launches = launch_count();
    if launches == 0 {
        return 0.0;
    }
    (infantry_near_hit_count() as f32) / (launches as f32)
}
