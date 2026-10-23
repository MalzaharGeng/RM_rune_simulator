mod dataset;
mod handler;
mod robomaster;
mod ros2;
mod shotlog;
mod spin;
mod statistic;
mod util;

use crate::dataset::prelude::DatasetPlugin;
use crate::ros2::plugin::{AutoAimBridge, ROS2Plugin, bevy_rotation_from_ros, mesh_world_centroid};
use crate::util::bevy::insert_all_child;
use crate::{
    handler::{on_activate, on_hit},
    robomaster::power_rune::{PowerRunePlugin, PowerRuneRoot, Projectile},
    statistic::{
        accurate_count, accurate_pct, increase_infantry_hit, increase_infantry_near_hit,
        increase_launch, infantry_hit_count, infantry_hit_pct, infantry_near_hit_count,
        infantry_near_hit_pct, launch_count,
    },
};
use avian3d::prelude::*;
use bevy::asset::embedded_asset;
use bevy::camera::Exposure;
use bevy::light::light_consts::lux;
use bevy::render::view::screenshot::{Capturing, Screenshot, save_to_disk};
use bevy::window::{CursorIcon, PresentMode, SystemCursorIcon};
use bevy::winit::WinitWindows;
use bevy::{
    anti_alias::fxaa::Fxaa,
    input::mouse::MouseMotion,
    prelude::*,
    scene::{SceneInstance, SceneInstanceReady},
};
use std::collections::{HashMap, HashSet, VecDeque};
use bevy::camera::visibility::RenderLayers;
use bevy::mesh::VertexAttributeValues;

#[derive(Component)]
struct MainCamera {
    follow_offset: Vec3,
}

#[derive(Component)]
struct LocalInfantry;

#[derive(Component)]
struct InfantryRoot;

#[derive(Resource, PartialEq)]
struct CameraMode(pub FollowingType);

#[derive(PartialEq)]
enum FollowingType {
    Free,
    Robot,
    ThirdPerson,
}

#[derive(Component, Default)]
struct InfantryChassis {
    yaw: f32,
}

#[derive(Component, Default)]
struct InfantryGimbal {
    local_yaw: f32,
    pitch: f32,
}

#[derive(Component)]
struct InfantryViewOffset(Transform);

#[derive(Component)]
struct InfantryLaunchOffset(Transform);

#[derive(PhysicsLayer, Default, Clone, Copy, Debug)]
enum GameLayer {
    #[default]
    Default,
    Vehicle,
    ProjectileSelf,
    ProjectileOther,
    Environment,
}

#[derive(Resource)]
struct Cooldown(Timer);

/// Creates help text at the bottom of the screen.
fn create_help_text() -> Text {
    format!(
        "total={} accurate={} pct={} infantry_hits={} infantry_pct={}\ninfantry_near={} infantry_near_pct={}  (near-side plate only = honest metric)\nControls: F2-Screenshot F3-Change Camera | WASD-Move Mouse-Look Space-Shoot\n2025 Actor&Thinker",
        launch_count(),
        accurate_count(),
        accurate_pct(),
        infantry_hit_count(),
        infantry_hit_pct(),
        infantry_near_hit_count(),
        infantry_near_hit_pct()
    )
        .into()
}

/// Spawns the help text at the bottom of the screen.
fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        create_help_text(),
        Node {
            position_type: PositionType::Absolute,
            bottom: px(12),
            left: px(12),
            ..default()
        },
    ));
}

fn update_help_text(mut text: Query<&mut Text>) {
    for mut text in text.iter_mut() {
        *text = create_help_text();
    }
}

struct EmbeddedAssetPlugin;

impl Plugin for EmbeddedAssetPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "assets/GROUND_DISPLAY.glb");
        embedded_asset!(app, "assets/POWER.glb");
        embedded_asset!(app, "assets/projectile_launch.ogg");
        embedded_asset!(app, "assets/rune_activated.ogg");
        embedded_asset!(app, "assets/vehicle.glb");
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoNoVsync,
                    name: Some("RoboMaster Simulator | Actor&Thinker".to_string()),
                    title: "RoboMaster Simulator | Actor&Thinker".to_string(),
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            EmbeddedAssetPlugin,
            PhysicsPlugins::default(),
        ))
        .add_plugins(ROS2Plugin::default())
        .add_plugins(PowerRunePlugin)
        .add_plugins(DatasetPlugin)
        .insert_resource(CameraMode(FollowingType::Robot))
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(SubstepCount(10))
        // 开火冷却 = 最小射击间隔。真机 20 发/秒 => 0.05 s；
        // 仿真原值 0.1 s 只有 10 发/秒，会把开火占空比人为砍半，掩盖自瞄侧的开火逻辑。
        // 对齐真机：0.1 -> 0.05。
        .insert_resource(Cooldown(Timer::from_seconds(0.05, TimerMode::Once)))
        // 敌方步兵「变速小陀螺」曲线配置 (可用 SPIN_* 环境变量覆盖)
        .insert_resource(spin::SpinConfig::from_env())
        .add_systems(Startup, (setup, setup_projectile))
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(on_hit)
        .add_observer(on_activate)
        .add_observer(count_infantry_hit)
        .add_systems(
            Update,
            (
                update_help_text,
                log_hit_stats,
                following_controls,
                vehicle_controls,
                remote_vehicle_controls,
                gimbal_controls,
                remote_gimbal_controls,
                apply_auto_aim_gimbal.after(gimbal_controls),
                freecam_controls,
                update_camera_follow,
                screenshot_on_f2,
                screenshot_saving,
            ),
        )
        // 敌方自转必须排在 TransformSystems::Propagate 之前:
        //   - 让本帧写入的 Transform 参与本帧的 GlobalTransform 传播 (装甲板位置与曲线同步)
        //   - capture_rune 约束在 Propagate 之后, 于是它读 SpinState 时本帧曲线已推进
        //     (否则两者无序, 真值偶尔会读到上一帧的角度 -> 重复帧 + 方位角滞后)
        .add_systems(
            PostUpdate,
            (
                enemy_infantry_move.before(TransformSystems::Propagate),
                pin_local_infantry.before(TransformSystems::Propagate),
            ),
        )
        .add_systems(
            PostUpdate,
            (
                projectile_launch.after(TransformSystems::Propagate),
                projectile_cleanup.after(projectile_launch),
            ),
        )
        .run();
}

#[derive(Component)]
struct PreciousCollision(
    HashMap<String, (ColliderConstructorHierarchy, CollisionLayers, Visibility)>,
);

/// 敌方步兵的固定水平位置 (Bevy 世界 XZ 平面)。
///
/// 默认放在**我方步兵出生点正前方 4 m**。我方车体初始朝向是 Bevy 的 -Z
/// (`chassis_data.yaw = 0` -> 单位四元数 -> forward = -Z; 枪口节点
/// `SHOT_DIRECTION` 也在 -Z 侧), 出生点是 (0, 1, 0), 所以正前方 4 m = (0, -4)。
///
/// 可用 `ENEMY_X` / `ENEMY_Z` 覆盖, 便于不重编译就换距离/方位。
fn enemy_fixed_xz() -> (f32, f32) {
    let get = |key: &str, default: f32| {
        std::env::var(key)
            .ok()
            .and_then(|s| s.trim().parse::<f32>().ok())
            .filter(|v| v.is_finite())
            .unwrap_or(default)
    };
    (get("ENEMY_X", 0.0), get("ENEMY_Z", -4.0))
}

/// 是否把我方步兵的水平位置钉在出生点。默认开 (`PIN_LOCAL_INFANTRY=0` 关)。
fn pin_local_enabled() -> bool {
    match std::env::var("PIN_LOCAL_INFANTRY") {
        Ok(v) => {
            let v = v.trim();
            !(v == "0" || v.eq_ignore_ascii_case("false") || v.eq_ignore_ascii_case("off"))
        }
        Err(_) => true,
    }
}

fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    spin_cfg: Res<spin::SpinConfig>,
) {
    spawn_text(&mut commands);
    let (enemy_x, enemy_z) = enemy_fixed_xz();
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.9, 0.95, 1.0),
            shadows_enabled: true,
            illuminance: lux::DIRECT_SUNLIGHT,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let layer_env = CollisionLayers::new(
        [GameLayer::Environment],
        [
            GameLayer::Default,
            GameLayer::Vehicle,
            GameLayer::ProjectileSelf,
            GameLayer::ProjectileOther,
        ],
    );

    let trimesh = ColliderConstructorHierarchy::new(
        ColliderConstructor::TrimeshFromMeshWithConfig(TrimeshFlags::all()),
    );
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
    };

    commands.spawn((
        SceneRoot(
            asset_server
                .load("embedded://daedalus/assets/GROUND_DISPLAY.glb#Scene0"),
        ),
        Transform::IDENTITY,
        PreciousCollision(HashMap::from([(
            "GROUND_LOW".to_string(),
            (trimesh.clone(), layer_env, Visibility::Hidden),
        )])),
    ));

    let mut power_rune_col = HashMap::from([]);
    for i in 1..=2 {
        /*power_rune_col.insert(
            format!("FACE_{}", i).to_string(),
            (trimesh.clone(), layer_env, Visibility::Visible),
        );*/
        for j in 1..=5 {
            for k in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                power_rune_col.insert(
                    format!("FACE_{}_TARGET_{}_{}", i, j, k).to_string(),
                    (voxel(0.015), layer_env, Visibility::Visible),
                );
            }
        }
    }
    commands.spawn((
        RigidBody::Static,
        CollisionMargin(0.001),
        Restitution::ZERO,
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/POWER.glb#Scene0"),
        ),
        Transform::IDENTITY,
        PowerRuneRoot,
        PreciousCollision(power_rune_col),
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
        CollisionMargin(0.001),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0"),
        ),
        Transform::from_xyz(0.0, 1.0, 0.0),
        InfantryRoot,
        LocalInfantry,
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0"),
        ),
        Transform::from_xyz(enemy_x, 1.0, enemy_z),
        InfantryRoot,
        // 变速小陀螺状态: 仅敌方步兵持有, 本地步兵 (带 LocalInfantry) 不参与
        spin::SpinState::from_config(&spin_cfg),
    ));

    commands.spawn((
        Camera3d::default(),
        Camera::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::PI / 180.0 * 45.0,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Exposure::SUNLIGHT,
        Msaa::Off,
        Fxaa::default(),
        RenderLayers::from_layers(&[0, 1]),
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::new(0.0, 3.0, 2.0),
        },
        ros2::plugin::MainCamera,
    ));
}

#[derive(Component, Clone)]
pub struct Armor(String);

#[derive(Component, Clone)]
pub struct EnemyArmor;

/// 命中计数。
///
/// 记两个数:
/// - `infantry_hits`: 碰到任意一块敌方装甲板就算 (原有语义, 保持不变)
/// - `infantry_near`: 额外要求被击中的那块板的**外法线朝向炮口**,
///   也就是自瞄本来打算打的那一块。这才是可信的指标。
///
/// 为什么需要后者: 仿真的命中判定被几何放大了。弹丸的碰撞层是 `ProjectileSelf`,
/// 而车体圆柱的 filter 不含它 (两边判定是双向与), 所以弹丸**直接穿过车体**;
/// 装甲板的碰撞体来自 `ColliderConstructorHierarchy`, 取 `CollisionLayers::DEFAULT`,
/// 与弹丸能相交 —— 于是弹丸穿堂而过时, 入射侧和出射侧都可能撞到板。
/// 又因为 4 块板间隔精确 90°、板宽 135.7mm + 弹丸 44.5mm 让切向角容差到 ±24.7°,
/// 「打错一块板 (差 90°)」和「穿堂打到对面板」都照样算命中。
///
/// 蒙特卡洛 (tools/hit_prob_model.py) 的结论: yaw 误差完全随机时总命中率仍有
/// 约 0.79, 而健康估计是 1.00 —— 只差 20 个点, 区分不出好坏。
/// 只算近侧命中后变成 1.00 vs 约 0.5, 才有区分度。
fn count_infantry_hit(
    event: On<CollisionStart>,
    projectiles: Query<(), With<Projectile>>,
    enemy_armors: Query<(&GlobalTransform, &Mesh3d), With<EnemyArmor>>,
    armor_meshes: Res<Assets<Mesh>>,
    enemy_center: Query<&GlobalTransform, (With<InfantryRoot>, Without<LocalInfantry>)>,
    gimbal: Query<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    shot_ids: Query<&shotlog::ShotId>,
    armor_names: Query<&Armor>,
    shotlog: Res<shotlog::ShotLog>,
    mut scored_projectiles: Local<HashSet<Entity>>,
) {
    let projectile = if projectiles.contains(event.collider1)
        && enemy_armors.contains(event.collider2)
    {
        Some((event.collider1, event.collider2))
    } else if projectiles.contains(event.collider2) && enemy_armors.contains(event.collider1) {
        Some((event.collider2, event.collider1))
    } else {
        None
    };

    let Some((projectile, armor)) = projectile else {
        return;
    };
    if !scored_projectiles.insert(projectile) {
        return;
    }
    increase_infantry_hit();

    // 近侧判定: 板心相对车体中心的方向就是板的外法线; 它朝向云台 => 自瞄瞄的就是这块。
    // 板心必须走 mesh 顶点质心, 节点原点是退化的 (见 plugin.rs::mesh_world_centroid)。
    let mut near = false;
    let plate_name = armor_names
        .get(armor)
        .map(|a| a.0.clone())
        .unwrap_or_else(|_| "?".to_string());
    if let Ok((armor_gt, armor_mesh)) = enemy_armors.get(armor) {
        if let (Some(center_gt), Some(gimbal_gt)) =
            (enemy_center.iter().next(), gimbal.iter().next())
        {
            if let Some(plate) = armor_meshes
                .get(armor_mesh)
                .and_then(|m| mesh_world_centroid(armor_gt, m))
            {
                let outward = (plate - center_gt.translation()).normalize_or_zero();
                let to_gimbal = (gimbal_gt.translation() - plate).normalize_or_zero();
                if outward.dot(to_gimbal) > 0.0 {
                    near = true;
                    increase_infantry_near_hit();
                }
            }
        }
    }

    // 逐发埋点: 记录这一发命中了哪块板 / 是否近侧。
    if let Ok(sid) = shot_ids.get(projectile) {
        shotlog.on_hit(sid.0, &plate_name, near);
    }
}

/// 装甲板内倾角 (度)。
///
/// 真车的装甲板不是竖直的: 板顶向车体中心倒 (内倾)。自瞄 `solver.cpp::reproject_armor`
/// 也是按 15 度内倾建模的 —— 它的装甲板局部系 +X 是**内法线** (指向车体中心),
/// 这点由 target.cpp 的量测模型确定 (`z[3] = ypr_in_world[0]` 直接与
/// `h(x)[3] = x[6] + id*90deg` 比较, 而 `h_armor_xyz` 给出
/// `armor_xy = center_xy - r*(cos a, sin a)`, 即 a 是从板指向车体中心的方位角)。
///
/// 仿真里原来的板是竖直的 (pitch = 0), 与真车不一致, 于是测试台低估了自瞄的模型误差。
/// 本项把它改成内倾。用 `ARMOR_TILT_DEG` 覆盖, 设 0 可完全关闭 (恢复竖直)。
fn armor_tilt_deg() -> f32 {
    std::env::var("ARMOR_TILT_DEG")
        .ok()
        .and_then(|s| s.trim().parse::<f32>().ok())
        .filter(|v| v.is_finite())
        .unwrap_or(15.0)
}

/// 仿真装甲板 `_P` 代理面的目标尺寸 (宽 = 水平, 高 = 竖直), 单位 m。
///
/// 取值来自 `vehicle.glb` 里**真正渲染出来的灯条网格** `ARMOR_n_L` 的包围盒
/// (外缘间距 0.13780 m, 灯条长 0.05984 m) —— 那才是车模 CAD 的真实尺寸。
/// 而 `ARMOR_n_P` 只有 0.13571 x 0.0500 / 0.0518 m, 比它矮 16%,
/// 却承担着碰撞体 / `[SIM_GT]` / 数据集标注 ⇒ 属于资产内部不一致, 这里归一到灯条尺寸。
///
/// 自瞄 `solver.cpp` 的 PnP 模型是 135.7 x 56 mm, 所以归一后仿真板会**略大于**
/// 自瞄模型 (水平 +1.5%, 竖直 +6.9%)。用 `ARMOR_PLATE_W` / `ARMOR_PLATE_H` 覆盖。
fn armor_plate_size() -> (f32, f32) {
    let read = |key: &str, default: f32| {
        std::env::var(key)
            .ok()
            .and_then(|s| s.trim().parse::<f32>().ok())
            .filter(|v| v.is_finite() && *v > 0.0)
            .unwrap_or(default)
    };
    (read("ARMOR_PLATE_W", 0.13780), read("ARMOR_PLATE_H", 0.05984))
}

fn setup_vehicle(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    root_query: Query<(Entity, Option<&LocalInfantry>), With<InfantryRoot>>,
    secondary_query: Query<&ChildOf, (Without<InfantryRoot>, Without<SceneInstance>)>,
    node_query: Query<
        (Entity, &Name, &ChildOf, &Transform, Option<&Mesh3d>),
        (Without<InfantryRoot>, Without<SceneInstance>),
    >,
    armor_meshes: Res<Assets<Mesh>>,
) {
    let root = events.entity;
    if root_query.get(root).is_err() {
        return;
    }
    let is_local = root_query.get(root).unwrap().1.is_some();

    let mut despawn = HashSet::new();

    for (node, name, &ChildOf(secondary), transform, _mesh) in node_query {
        let Ok(&ChildOf(root2)) = secondary_query.get(secondary) else {
            continue;
        };
        if root != root2 {
            continue;
        }
        despawn.insert(secondary);
        commands.entity(secondary).remove_child(node);
        commands.entity(root).add_child(node);
        let mut ent = commands.entity(node);
        if is_local {
            ent.insert(LocalInfantry);
        }
        match name.as_str() {
            "BASE" => {
                ent.insert(InfantryChassis::default());
                let mut stack = VecDeque::from([(node, name)]);
                let mut set = HashSet::new();
                let mut tilted: Vec<String> = Vec::new();
                let mut sized: Vec<String> = Vec::new();
                let mut am_tilt: Vec<(Entity, Vec3, Quat)> = Vec::new();
                while let Some((e, name)) = stack.pop_front() {
                    if !set.insert(e) {
                        continue;
                    }
                    if name.starts_with("ARMOR_") && name.ends_with("_P") {
                        insert_all_child(&mut commands, e, &children, || Armor(name.to_string()));
                        if !is_local {
                            insert_all_child(&mut commands, e, &children, || EnemyArmor);
                        }
                        commands.entity(e).insert(ColliderConstructorHierarchy::new(
                            ColliderConstructor::TrimeshFromMeshWithConfig(
                                TrimeshFlags::MERGE_DUPLICATE_VERTICES,
                            ),
                        ));

                        // 板顶向车体中心内倾 + 把板面尺寸归一到真实装甲板。
                        //
                        // ⚠️ 两个坑:
                        //  (1) 板几何烘在网格里 (ARMOR_n_P 相对 BASE 的合成变换是单位矩阵),
                        //      所以必须绕**板心**做变换, 不能直接改 rotation / scale
                        //      (那会绕节点原点 = 车体中心甩出去)。
                        //  (2) Bevy 0.17 把 glb 节点的 mesh 放在**子实体**上, ARMOR_n_P 自己
                        //      没有 Mesh3d, 所以板心与现有尺寸要遍历整棵子树、把所有 mesh 顶点求出来。
                        //
                        // 内倾施加在**父节点 AM_n** 上 (而不是只转 _P), 这样灯条 ARMOR_n_L、
                        // 编号 ARMOR_n_3_C、模块 ARMOR_n 会跟着一起内倾 —— 否则画面里板是竖直的、
                        // 真值却是 15 度内倾的, 两者对不上。
                        // AM 局部系里: 板心 C = t_p + c (c = 网格顶点质心, 网格坐标即 BASE 坐标),
                        // 绕 C 转 R 得  AM' = T(C) * R * T(-C) * AM
                        //   =>  t_am' = C + R*(t_am - C),  R' = R
                        // _P 自己只做"绕板心缩放": t_p' = t_p + c - S∘c, R = I, S = 各轴缩放。
                        //
                        // 尺寸: GLB 里 ARMOR_n_P 只有 135.71 x 50.0 / 51.8 mm, 比实际渲染出来的
                        // 灯条 (ARMOR_n_L, 137.80 x 59.84 mm) 矮 16% —— 而碰撞体、[SIM_GT]
                        // 与数据集标注都取这个 _P 面。这里把它归一到灯条外缘矩形, 让仿真自己的
                        // 真值与画面一致。自瞄模型是 135.7 x 56 mm, 故仿真板会略大于自瞄模型。
                        let (target_w, target_h) = armor_plate_size();
                        {
                            let mut sum = Vec3::ZERO;
                            let mut count = 0usize;
                            let mut min_v = Vec3::splat(f32::INFINITY);
                            let mut max_v = Vec3::splat(f32::NEG_INFINITY);
                            let mut stack2: VecDeque<Entity> = VecDeque::new();
                            stack2.push_back(e);
                            let mut seen2: HashSet<Entity> = HashSet::new();
                            while let Some(x) = stack2.pop_front() {
                                if !seen2.insert(x) {
                                    continue;
                                }
                                if let Ok((_, _, _, _, Some(m))) = node_query.get(x) {
                                    if let Some(mesh) = armor_meshes.get(&m.0) {
                                        for (_a, values) in mesh.attributes() {
                                            if let VertexAttributeValues::Float32x3(v) = values {
                                                for &p in v {
                                                    let p = Vec3::from(p);
                                                    sum += p;
                                                    min_v = min_v.min(p);
                                                    max_v = max_v.max(p);
                                                    count += 1;
                                                }
                                                break;
                                            }
                                        }
                                    }
                                }
                                if let Ok(ch) = children.get(x) {
                                    for c in ch.iter() {
                                        stack2.push_back(c);
                                    }
                                }
                            }

                            if count == 0 {
                                bevy::log::warn!("[ARMOR_SIZE] {} no mesh vertices", name);
                            } else if let Ok((
                                _,
                                _,
                                &ChildOf(am),
                                &Transform { translation: t_p, .. },
                                _,
                            )) = node_query.get(e)
                            {
                                let c = sum / count as f32;
                                let ext = max_v - min_v;
                                // 最薄的一维是板厚 (不缩放); 竖直恒为 Y; 剩下一维是水平
                                let mut thin = 0usize;
                                if ext[1] < ext[thin] {
                                    thin = 1;
                                }
                                if ext[2] < ext[thin] {
                                    thin = 2;
                                }
                                let horiz = if thin == 0 {
                                    Some(2usize)
                                } else if thin == 2 {
                                    Some(0usize)
                                } else {
                                    None
                                };
                                let mut s = Vec3::ONE;
                                if let Some(h) = horiz {
                                    if ext[h] > 1e-6 {
                                        s[h] = target_w / ext[h];
                                    }
                                }
                                if thin != 1 && ext[1] > 1e-6 {
                                    s[1] = target_h / ext[1];
                                }

                                // (a) _P 代理面: 只绕板心缩放, 不旋转 (旋转交给父节点)
                                commands.entity(e).insert(Transform {
                                    translation: t_p + c - s * c,
                                    rotation: Quat::IDENTITY,
                                    scale: s,
                                });

                                // (b) 父节点 AM_n: 绕板心内倾 (板法线方向朝车体中心)
                                let tilt = armor_tilt_deg();
                                let outward = Vec3::new(c.x, 0.0, c.z).normalize_or_zero();
                                let rot = if tilt.abs() > 1e-6 && outward.length_squared() > 0.5 {
                                    tilted.push(name.to_string());
                                    Quat::from_axis_angle(outward.cross(Vec3::Y), tilt.to_radians())
                                } else {
                                    if tilt.abs() > 1e-6 {
                                        bevy::log::warn!("[ARMOR_TILT] {} outward degenerate", name);
                                    }
                                    Quat::IDENTITY
                                };
                                if let Ok((_, _, _, &Transform { translation: t_am, .. }, _)) =
                                    node_query.get(am)
                                {
                                    // 内倾支点必须用 **BASE 系** 的真实板心, 而不是 AM 局部系。
                                    //
                                    // 世界位置: world(v) = t_am + t_p + v (v = mesh 顶点原始坐标)。
                                    //   - AM_1 / AM_2: t_am = 0, t_p = 0  => BASE 系板心 = c, 与旧写法等价。
                                    //   - AM_3 / AM_4: t_am != 0, 且 t_p = -t_am (资产用来抵消父平移)
                                    //                  => BASE 系板心 = t_am + t_p + c = c。
                                    //   旧写法 c_local = t_p + c = c - t_am 会把支点甩到车体中心附近,
                                    //   导致 AM_3/AM_4 绕车心转 15°、板心竖直错位约 57mm (2026-10-13 查实的 BUG)。
                                    let c_world = t_am + t_p + c;
                                    am_tilt.push((am, c_world + rot * (t_am - c_world), rot));
                                }

                                if let Some(h) = horiz {
                                    sized.push(format!(
                                        "{} {:.2}x{:.2} -> {:.1}x{:.1}mm",
                                        name,
                                        ext[h] * 1000.0,
                                        ext[1] * 1000.0,
                                        ext[h] * s[h] * 1000.0,
                                        ext[1] * s[1] * 1000.0
                                    ));
                                }
                            }
                        }
                    }
                    for (ee, n, &ChildOf(r), _, _) in node_query {
                        if r == e {
                            stack.push_back((ee, n));
                        }
                    }
                }
                // 内倾统一在父节点 AM_n 上落一次 (每个 AM 只有一个 _P 子节点)
                for (am, t, rot) in am_tilt {
                    commands.entity(am).insert(Transform {
                        translation: t,
                        rotation: rot,
                        scale: Vec3::ONE,
                    });
                }
                if !tilted.is_empty() {
                    bevy::log::info!(
                        "[ARMOR_TILT] {:.1} deg inward tilt applied to {} whole assembly(AM_n): {}",
                        armor_tilt_deg(),
                        tilted.len(),
                        tilted.join(", ")
                    );
                }
                if !sized.is_empty() {
                    bevy::log::info!(
                        "[ARMOR_SIZE] _P proxy resized to {:.2} x {:.2} mm: {}",
                        armor_plate_size().0 * 1000.0,
                        armor_plate_size().1 * 1000.0,
                        sized.join(", ")
                    );
                }
            }
            "GIMBAL" => {
                ent.insert(InfantryGimbal { local_yaw: 0.0, pitch: 0.0 });
            }
            "SHOT_DIRECTION" => {
                ent.insert(InfantryLaunchOffset(transform.clone()));
            }
            "CAM_DIRECTION" => {
                ent.insert(InfantryViewOffset(transform.clone()));
            }
            _ => {}
        }
    }

    for ent in despawn {
        commands.entity(ent).despawn();
    }
}

fn setup_projectile(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(ProjectileSetting(
        meshes.add(Sphere::new(44.5 * 0.001 / 2.0)),
        materials.add(StandardMaterial {
            base_color: Color::srgba(0.132866, 1.0, 0.132869, 0.55),
            emissive: LinearRgba::new(0.132866, 1.0, 0.132869, 0.55),
            emissive_exposure_weight: -1.0,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
    ));
}

#[derive(Component)]
struct ProjectileLifetime(Timer);

#[derive(Resource)]
struct ProjectileSetting(Handle<Mesh>, Handle<StandardMaterial>);

fn projectile_launch(
    time: Res<Time>,
    _asset_server: Res<AssetServer>,
    mut cooldown: ResMut<Cooldown>,
    mut commands: Commands,
    auto_aim: Res<AutoAimBridge>,
    setting: Res<ProjectileSetting>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&InfantryLaunchOffset, With<LocalInfantry>>,
    shotlog: Res<shotlog::ShotLog>,
) {
    cooldown.0.tick(time.delta());
    if !cooldown.0.is_finished() {
        return;
    }
    cooldown.0.reset();
    let auto_fire = auto_aim.take_fire_request();
    if keyboard.pressed(KeyCode::Space) || auto_fire {
        increase_launch();
        let direction = gimbal.0.rotation().mul_vec3(Vec3::NEG_Z).normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        // 逐发埋点: 记离膛时刻与"距最近一次 fire 请求"的延迟。
        // 姿态对比留给 9871 遥测(plan_yaw vs gimbal_yaw_state), 这里不重复记。
        let dist = auto_aim
            .latest_command()
            .map(|c| c.distance)
            .unwrap_or(f32::NAN);
        let shot_id = shotlog.on_launch(dist);
        let vel = infantry.1.0 + direction * 25.0;
        // ⚠️ Bevy 的 Bundle 元组最多 16 个元素; 原 16 个已满, 所以 shot_id 用 .insert 追加。
        commands
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(44.5 * 0.001 / 2.0),
                Mass(44.5 * 0.001),
                Friction::new(1.1),
                Restitution::ZERO,
                LinearDamping(0.05),
                CollisionLayers::new(
                    GameLayer::ProjectileSelf,
                    [
                        GameLayer::Default,
                        GameLayer::Vehicle,
                        GameLayer::ProjectileSelf,
                        GameLayer::ProjectileOther,
                        GameLayer::Environment,
                    ],
                ),
                Mesh3d(setting.0.clone()),
                MeshMaterial3d(setting.1.clone()),
                LinearVelocity(vel),
                AngularVelocity(infantry.2.0),
                Transform::IDENTITY.with_translation(
                    infantry.0.translation + (gimbal.0.rotation() * launch_offset.0.translation),
                ),
                //AudioPlayer::new(asset_server.load("projectile_launch.ogg")),
                Projectile,
                RenderLayers::layer(1),
                ProjectileLifetime(Timer::from_seconds(PROJECTILE_LIFETIME_SECS, TimerMode::Once)),
            ))
            .insert(shot_id);
    }
}

fn projectile_cleanup(
    time: Res<Time>,
    mut commands: Commands,
    mut projectiles: Query<(Entity, &mut ProjectileLifetime, &shotlog::ShotId), With<Projectile>>,
    shotlog: Res<shotlog::ShotLog>,
) {
    for (entity, mut lifetime, shot_id) in &mut projectiles {
        if lifetime.0.tick(time.delta()).just_finished() {
            // 寿命耗尽仍未命中 => 漏弹。
            shotlog.on_miss(shot_id.0);
            commands.entity(entity).despawn();
        }
    }
}

fn setup_collision(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name, With<Children>>,
    root_query: Query<(Entity, &PreciousCollision)>,
) {
    let Ok((_, PreciousCollision(map))) = root_query.get(events.entity) else {
        return;
    };
    for e in children.iter_descendants(events.entity) {
        let Ok(name) = name.get(e) else {
            continue;
        };
        if let Some((constructor, layer, visibility)) = map.get(&name.to_string()) {
            commands.entity(e).insert((
                RigidBody::Static,
                Mass(0.0),
                Restitution::ZERO,
                constructor.clone(),
                CollisionMargin(0.001),
                *layer,
            ));
            if visibility == Visibility::Hidden {
                commands.entity(e).insert(*visibility);
            }
        }
    }
    commands.entity(events.entity).remove::<PreciousCollision>();
}

// 单位 m/s^2
const VEHICLE_ACCEL: f32 = 10.0;
// 单位 rad/s
const VEHICLE_ROTATION_SPEED: f32 = 3.0;
// 云台转速上限。真机云台实际约 6~8 rad/s。
// 原值 3.0 时，小陀螺换板阶跃（实测中位 0.114 rad）单拍追不上，
// |plan_yaw - gimbal_yaw| 长期超 fire_thresh，导致近距离开火占空比只有 ~3%。
// 6.0 rad/s @60fps => 单拍 0.10 rad，可基本一拍吃下换板阶跃。
const GIMBAL_ROTATION_SPEED: f32 = 6.0;

const MAX_VEHICLE_VELOCITY: f32 = 6.0;
// 弹丸存活时长，单位 s。
//
// ⚠️ 2026-10-12 从 10.0 降到 2.0。原注释说「远大于 45° 最大滞空 3.61s」，
// 但那是按"弹丸少"的前提写的。删掉仿真入口那道多余的开火门之后，自瞄开火占空比
// 从 2% 升到 ~52%，配合 20 发/秒 => 场上同时存在 20 × 10 = 200 个动态刚体，
// 而弹丸的碰撞层含 ProjectileSelf 自碰撞（碰撞对 O(n²)）⇒ 实测仿真帧率
// 从 ~85 Hz 崩到 ~20 Hz（见 _pj_analysis/fps_analyze.py 的帧率曲线），
// 自瞄图像流断续、tracker 反复 "Large dt ... reset target"，重投影严重乱飘。
// 2.0 s @ 25 m/s = 50 m 射程，平射场景（敌人 4 m）绰绰有余，弹丸数降到 ~40。
const PROJECTILE_LIFETIME_SECS: f32 = 2.0;

fn vehicle_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<(Forces, &Mass), (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (With<LocalInfantry>, Without<InfantryGimbal>),
    >,
) {
    if mode.0 == FollowingType::Free {
        return;
    }

    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyW) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let (mut forces, mass) = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_impulse(mass.0 * dt * desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_impulse(mass.0 * dt * -brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_impulse(mass.0 * dt * -linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyQ) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyE) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

fn remote_vehicle_controls(
    time: Res<Time>,
    _mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<Forces, (With<InfantryRoot>, Without<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (Without<LocalInfantry>, Without<InfantryGimbal>),
    >,
) {
    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyI) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyK) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyL) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let mut forces = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_acceleration(desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_acceleration(-brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_acceleration(-linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyU) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyO) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

fn gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if keyboard.pressed(KeyCode::ArrowLeft) {
        gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowRight) {
        gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowUp) {
        gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::ArrowDown) {
        gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
    }

    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

fn apply_auto_aim_gimbal(
    time: Res<Time>,
    auto_aim: Res<AutoAimBridge>,
    parent_transforms: Query<&GlobalTransform>,
    gimbal: Single<
        (Entity, &ChildOf, &mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let Some(command) = auto_aim.latest_command() else {
        return;
    };
    if command.distance < 0.0 {
        return;
    }

    let (_gimbal_entity, parent, mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();
    let Ok(parent_transform) = parent_transforms.get(parent.0) else {
        return;
    };

    // 协议里的 yaw/pitch 是 odom_vision 绝对角（度）；这里先构造 ROS 朝向，再转回 Bevy 局部朝向。
    let desired_ros_rotation = Quat::from_euler(EulerRot::ZYX, command.yaw, -command.pitch, 0.0);
    let desired_bevy_world_rotation = bevy_rotation_from_ros(desired_ros_rotation);
    let local_rotation = parent_transform.rotation().inverse() * desired_bevy_world_rotation;
    let (desired_local_yaw, desired_local_pitch, _) = local_rotation.to_euler(EulerRot::YXZ);

    let max_step = GIMBAL_ROTATION_SPEED * time.delta_secs();
    let yaw_error = (desired_local_yaw - gimbal_data.local_yaw + std::f32::consts::PI)
        .rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;
    let pitch_error = desired_local_pitch - gimbal_data.pitch;

    gimbal_data.local_yaw += yaw_error.clamp(-max_step, max_step);
    gimbal_data.pitch += pitch_error.clamp(-max_step, max_step);
    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);
    gimbal_transform.rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
}

fn remote_gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if keyboard.pressed(KeyCode::KeyC) {
        gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyB) {
        gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyF) {
        gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyV) {
        gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
    }

    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

fn following_controls(mut mode: ResMut<CameraMode>, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::F3) {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

fn update_camera_follow(
    camera_query: Single<(&mut Transform, &MainCamera), Without<LocalInfantry>>,
    infantry: Single<&Transform, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<&Transform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
    mode: Res<CameraMode>,
) {
    let gimbal_transform = gimbal.into_inner();
    let (mut camera_transform, camera_offset) = camera_query.into_inner();

    match mode.0 {
        FollowingType::Robot => {
            camera_transform.translation = infantry.translation
                + (infantry.rotation * gimbal_transform.rotation) * view_offset.0.translation;
            camera_transform.rotation = infantry.rotation * gimbal_transform.rotation;
        }
        FollowingType::ThirdPerson => {
            let base_transform = infantry.into_inner();
            let offset = base_transform.rotation * camera_offset.follow_offset;
            camera_transform.translation = base_transform.translation + offset;
            camera_transform.look_at(base_transform.translation, Vec3::Y);
        }
        FollowingType::Free => {}
    }
}

/// 敌方步兵「变速小陀螺」。
///
/// 与原实现相比:
/// - **取消水平方向移动**: 不再做正弦往返, 而是把 x/z 钉在 [`enemy_fixed_xz`]
///   给出的固定点 (默认 = 我方出生点正前方 4 m, 即 (0, -4)),
///   使敌方原地固定 (y 仍由物理/重力决定, 保证正常贴地)。钉住位置也可避免被弹丸冲量顶走。
/// - **改为变速旋转**: 转角由 [`spin::SpinState`] 累计推进 (固定角加速度 + 变速区间),
///   不再是恒定的 `3.0 * t`。累计转角连续不重置, 保证姿态无跳变。
///
/// 时间用仿真时间 (`Time`), 暂停时曲线同步停止。
///
/// **注意**: 敌方车体是 avian3d `RigidBody::Dynamic`, 姿态由物理回写 (`Rotation` -> `Transform`)。
/// 只写 `Transform.rotation` 时, 物理回写会用上一固定步采样的 `Rotation` 覆盖它, 导致
/// 车体实际朝向比 `SpinState.angle` 滞后最多一个固定步 (64 Hz -> 15.6 ms, 18 rad/s 时约 10°),
/// 装甲板真值随之抖动。因此这里同时写 avian3d 的 `Rotation`, 让回写结果与曲线完全一致。
fn enemy_infantry_move(
    time: Res<Time>,
    spin_cfg: Res<spin::SpinConfig>,
    mut enemy: Query<
        (&mut Transform, &mut Rotation, &mut spin::SpinState),
        (With<InfantryRoot>, Without<LocalInfantry>),
    >,
) {
    let (fixed_x, fixed_z) = enemy_fixed_xz();

    let dt = time.delta_secs() as f64;
    let sim_t = time.elapsed_secs() as f64;
    for (mut transform, mut phys_rot, mut state) in enemy.iter_mut() {
        // 固定在原地: 只钉水平位置, y 交给物理 (贴地/重力行为不变)
        transform.translation.x = fixed_x;
        transform.translation.z = fixed_z;

        spin::advance_spin(&mut state, &spin_cfg, dt, sim_t);
        // 构造四元数时对 2π 取模: 姿态完全等价, 但可避免长时间运行后 f32 精度退化
        // (state.angle 本身保持连续累计, 真值输出的仍是完整转角)。
        // Bevy 绕 +Y 旋转 <-> ROS 系绕 +Z 的 yaw, 二者同号 (见 ros2::plugin::M_ALIGN_MAT3)
        let render_angle = state.angle.rem_euclid(std::f64::consts::TAU);
        let q = Quat::from_rotation_y(render_angle as f32);
        transform.rotation = q;
        // 同步写物理姿态, 否则物理回写会用滞后一个固定步的旧值覆盖上面的 Transform
        phys_rot.0 = q;
    }
}

/// 把我方步兵的水平位置钉在出生点 (Bevy (0, 0)), **直到用户第一次给出车体移动输入**。
///
/// 为什么需要: 我方车体是 `RigidBody::Dynamic` 且 `LinearDamping(0.0)`, 出生点 (0, 1, 0)
/// 又和 POWER 符 (原点) 重合。符是 `RigidBody::Static` 而且会转, 实测落地后车体会被挤出去、
/// 顺着地形滑走 —— 在一台跑了几十分钟的实例上量到它停在离出生点 **5.4 m** 的地方
/// (`/tf` 的 gimbal_link 在 odom_vision 下是 (-2.451, -4.791), 换算回 Bevy 是 (4.79, 2.45))。
/// 那样「敌方放在我方出生点前方 4 m」就不可复现 —— 实际距离会缩到 2.6 m 左右。
///
/// **只要用户在跟随视角下按下 W/A/S/D/Q/E 里任意一个, 钉住就永久解除**, 之后完全自由驾驶
/// (想重新钉住只能重启仿真)。这样开局几何依然确定、可复现, 又不妨碍手动挪车。
/// 想开局就自由: `PIN_LOCAL_INFANTRY=0`。
///
/// 注意: `FollowingType::Free` 视角下 W/A/S/D 是移动**自由相机** (见 `free_camera_controls`),
/// 不是开车, 所以那种情况下不解除。
///
/// 只钉水平位置, y 仍交给物理 (贴地/重力行为不变), 与 `enemy_infantry_move` 同机制。
fn pin_local_infantry(
    mut cached: Local<Option<bool>>,
    mut released: Local<bool>,
    mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut ours: Query<
        (&mut Transform, &mut Position, &mut LinearVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
) {
    if !*cached.get_or_insert_with(pin_local_enabled) {
        return;
    }
    if *released {
        return;
    }
    if mode.0 != FollowingType::Free {
        const MOVE_KEYS: [KeyCode; 6] = [
            KeyCode::KeyW,
            KeyCode::KeyS,
            KeyCode::KeyA,
            KeyCode::KeyD,
            KeyCode::KeyQ,
            KeyCode::KeyE,
        ];
        if MOVE_KEYS.iter().any(|k| keyboard.pressed(*k)) {
            *released = true;
            bevy::log::info!(
                "[PIN] local infantry released by WASD/QE -> free driving enabled"
            );
            return;
        }
    }
    for (mut transform, mut position, mut velocity) in ours.iter_mut() {
        transform.translation.x = 0.0;
        transform.translation.z = 0.0;
        // 同时钉物理位置与水平速度: 车体 `LinearDamping(0.0)`, 没有任何阻力,
        // 只钉 Transform 会被物理回写 (Position -> Transform) 覆盖, 照样滑走。
        position.0.x = 0.0;
        position.0.z = 0.0;
        velocity.0.x = 0.0;
        velocity.0.z = 0.0;
    }
}

/// 周期性把命中统计写进日志 (每 2 s 一条 `[HITSTAT]`)。
///
/// 仿真的 HUD 画在窗口里, 后台/无头运行时看不到, 也没法截图。
/// 这个日志把 HUD 上的四个数原样落到 stdout, 脚本就能直接判读命中率:
/// `hits` = 碰到任意一块敌方装甲板, `near` = 额外要求该板外法线朝向炮口
/// (自瞄本来打算打的那一块, 才是可信指标 —— 见 `statistic.rs` 的说明)。
fn log_hit_stats(mut last: Local<f32>, time: Res<Time>) {
    let now = time.elapsed_secs();
    if now - *last < 2.0 {
        return;
    }
    *last = now;
    bevy::log::info!(
        "[HITSTAT] t={:.1}s launches={} hits={} ({:.2}%) near={} ({:.2}%)",
        now,
        statistic::launch_count(),
        statistic::infantry_hit_count(),
        100.0 * statistic::infantry_hit_pct(),
        statistic::infantry_near_hit_count(),
        100.0 * statistic::infantry_near_hit_pct(),
    );
}

fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<InfantryRoot>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * 0.003;
        let new_pitch = (pitch - mouse_delta.y * 0.003).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    const CAMERA_SPEED: f32 = 8.0;
    let speed = CAMERA_SPEED * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}

fn screenshot_on_f2(
    mut commands: Commands,
    input: Res<ButtonInput<KeyCode>>,
    mut counter: Local<u32>,
) {
    if input.just_pressed(KeyCode::F2) {
        let path = format!("./screenshot-{}.png", *counter);
        *counter += 1;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}

fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}
