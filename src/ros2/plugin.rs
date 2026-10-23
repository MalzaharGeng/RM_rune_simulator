use crate::ros2::capture::{CaptureCamera, CaptureConfig, RosCaptureContext, RosCapturePlugin};
use crate::ros2::gt_udp::{GtUdpSender, SpinGt};
use crate::ros2::topic::*;
use crate::spin::{SpinConfig, SpinState};
use crate::{
    Armor, EnemyArmor, InfantryGimbal, InfantryViewOffset, LocalInfantry, arc_mutex, publisher,
    robomaster::power_rune::{PowerRune, RuneIndex},
};
use bevy::mesh::VertexAttributeValues;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use futures::{
    Future, executor::LocalPool, stream::Stream, stream::StreamExt, task::LocalSpawnExt,
};
use r2r::ClockType::SystemTime;
use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::{Clock, Context, Node, std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use serde_json::Value;
use std::f32::consts::PI;
use std::time::{Duration, Instant};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
};

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
fn align_quat() -> Quat {
    Quat::from_mat3(&M_ALIGN_MAT3)
}

#[inline]
pub fn ros_rotation_from_bevy(rotation: Quat) -> Quat {
    let align_quat = align_quat();
    align_quat * rotation * align_quat.inverse()
}

#[inline]
pub fn bevy_rotation_from_ros(rotation: Quat) -> Quat {
    let align_quat = align_quat();
    align_quat.inverse() * rotation * align_quat
}

#[inline]
pub fn transform(bevy_transform: Transform) -> r2r::geometry_msgs::msg::Transform {
    let new_rotation = ros_rotation_from_bevy(bevy_transform.rotation);
    let new_translation = M_ALIGN_MAT3 * bevy_transform.translation;
    r2r::geometry_msgs::msg::Transform {
        translation: r2r::geometry_msgs::msg::Vector3 {
            x: new_translation.x as f64,
            y: new_translation.y as f64,
            z: new_translation.z as f64,
        },
        rotation: r2r::geometry_msgs::msg::Quaternion {
            x: new_rotation.x as f64,
            y: new_rotation.y as f64,
            z: new_rotation.z as f64,
            w: new_rotation.w as f64,
        },
    }
}

#[inline]
pub fn pose_from_transform(bevy_transform: Transform) -> Pose {
    let ros_transform = transform(bevy_transform);
    Pose {
        position: r2r::geometry_msgs::msg::Point {
            x: ros_transform.translation.x,
            y: ros_transform.translation.y,
            z: ros_transform.translation.z,
        },
        orientation: ros_transform.rotation,
    }
}

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

const AUTO_AIM_CMD_TYPE: &str = "rm_interfaces/msg/GimbalCmd";
const SET_MODE_SERVICE_TYPE: &str = "rm_interfaces/srv/SetMode";
const AUTO_AIM_CMD_TIMEOUT: Duration = Duration::from_millis(200);
const AUTO_AIM_TOPIC: &str = "/rune_solver/cmd_gimbal";
const RUNE_DETECTOR_SET_MODE_SERVICE: &str = "/rune_detector/set_mode";
const RUNE_SOLVER_SET_MODE_SERVICE: &str = "/rune_solver/set_mode";
const SMALL_RUNE_RED_MODE: i64 = 2;
const ROOT_FRAME_ID: &str = "odom_vision";
const GIMBAL_FRAME_ID: &str = "gimbal_link";
const CAMERA_LINK_FRAME_ID: &str = "camera_link";
const CAMERA_OPTICAL_FRAME_ID: &str = "camera_optical_frame";

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct AutoAimCommand {
    pub pitch: f32,
    pub yaw: f32,
    pub yaw_diff: f32,
    pub pitch_diff: f32,
    pub distance: f32,
    pub fire_advice: bool,
}

#[derive(Debug, Default)]
struct AutoAimState {
    last_command: Option<AutoAimCommand>,
    last_received_at: Option<Instant>,
    last_topic: Option<&'static str>,
    pending_fire_at: Option<Instant>,
}

#[derive(Resource, Clone, Default)]
pub struct AutoAimBridge(Arc<Mutex<AutoAimState>>);

impl AutoAimBridge {
    fn update(&self, topic: &'static str, command: AutoAimCommand, shotlog: &crate::shotlog::ShotLog) {
        let mut state = self.0.lock().unwrap();
        let now = Instant::now();
        if command.distance < 0.0 {
            state.pending_fire_at = None;
        } else if command.fire_advice {
            state.pending_fire_at = Some(now);
            // 逐发埋点: 记下 fire 请求到达时刻(与离膛时刻对齐用)。
            shotlog.on_fire_request(command.yaw, command.pitch, command.distance);
        }
        state.last_command = Some(command);
        state.last_received_at = Some(now);
        state.last_topic = Some(topic);
    }

    pub fn latest_command(&self) -> Option<AutoAimCommand> {
        let state = self.0.lock().unwrap();
        let received_at = state.last_received_at?;
        if received_at.elapsed() > AUTO_AIM_CMD_TIMEOUT {
            return None;
        }
        state.last_command.clone()
    }

    pub fn take_fire_request(&self) -> bool {
        let mut state = self.0.lock().unwrap();
        let Some(pending_at) = state.pending_fire_at.take() else {
            return false;
        };
        let fresh_command = state
            .last_received_at
            .is_some_and(|received_at| received_at.elapsed() <= AUTO_AIM_CMD_TIMEOUT);
        fresh_command
            && pending_at.elapsed() <= AUTO_AIM_CMD_TIMEOUT
            && state.last_command.as_ref().is_some_and(|command| command.distance >= 0.0)
    }
}

fn parse_auto_aim_command(message: &Value) -> Option<AutoAimCommand> {
    let pitch = message.get("pitch")?.as_f64()? as f32;
    let yaw = message.get("yaw")?.as_f64()? as f32;
    let yaw_diff = message.get("yaw_diff")?.as_f64()? as f32;
    let pitch_diff = message.get("pitch_diff")?.as_f64()? as f32;
    let distance = message.get("distance")?.as_f64()? as f32;
    let fire_advice = message.get("fire_advice")?.as_bool()?;

    Some(AutoAimCommand {
        pitch: pitch.to_radians(),
        yaw: yaw.to_radians(),
        yaw_diff: yaw_diff.to_radians(),
        pitch_diff: pitch_diff.to_radians(),
        distance,
        fire_advice,
    })
}

fn spawn_auto_aim_subscription<S>(
    spawner: &futures::executor::LocalSpawner,
    topic: &'static str,
    mut subscription: S,
    auto_aim: AutoAimBridge,
    shotlog: crate::shotlog::ShotLog,
) where
    S: Stream<Item = r2r::Result<Value>> + Unpin + 'static,
{
    let result = spawner.spawn_local(async move {
        while let Some(message) = subscription.next().await {
            match message {
                Ok(message) => {
                    if let Some(command) = parse_auto_aim_command(&message) {
                        auto_aim.update(topic, command, &shotlog);
                    } else {
                        warn!("Malformed auto aim message on {topic}: {message}");
                    }
                }
                Err(err) => warn!("Auto aim subscriber error on {topic}: {err:?}"),
            }
        }
        warn!("Auto aim subscriber on {topic} stopped.");
    });

    if result.is_err() {
        error!("Failed to spawn auto aim subscription task for {topic}.");
    }
}

fn setup_auto_aim_subscriptions(
    node: &mut Node,
    spawner: &futures::executor::LocalSpawner,
    auto_aim: AutoAimBridge,
    shotlog: crate::shotlog::ShotLog,
) {
    match node.subscribe_untyped(
        AUTO_AIM_TOPIC,
        AUTO_AIM_CMD_TYPE,
        r2r::QosProfile::sensor_data(),
    ) {
        Ok(subscription) => {
            info!("Subscribed to {AUTO_AIM_TOPIC} for auto aim control.");
            spawn_auto_aim_subscription(spawner, AUTO_AIM_TOPIC, subscription, auto_aim, shotlog);
        }
        Err(err) => {
            warn!(
                "Auto aim topic {AUTO_AIM_TOPIC} is unavailable: {err:?}. \
Make sure rm_interfaces is sourced before building so r2r can generate GimbalCmd."
            );
        }
    }
}

async fn call_set_mode(
    service_name: &'static str,
    service_available: impl Future<Output = r2r::Result<()>>,
    client: r2r::ClientUntyped,
) {
    match service_available.await {
        Ok(()) => {}
        Err(err) => {
            warn!("Mode service {service_name} is not available: {err:?}");
            return;
        }
    }

    let request = serde_json::json!({ "mode": SMALL_RUNE_RED_MODE });
    match client.request(request) {
        Ok(response) => match response.await {
            Ok(Ok(payload)) => {
                info!("Configured {service_name} to mode {SMALL_RUNE_RED_MODE}: {payload}");
            }
            Ok(Err(err)) => {
                warn!("Service {service_name} returned a malformed JSON response: {err:?}");
            }
            Err(err) => {
                warn!("Failed waiting for {service_name} response: {err:?}");
            }
        },
        Err(err) => warn!("Failed to send request to {service_name}: {err:?}"),
    }
}

fn setup_mode_service_client(
    node: &mut Node,
    spawner: &futures::executor::LocalSpawner,
    service_name: &'static str,
) {
    match node.create_client_untyped(service_name, SET_MODE_SERVICE_TYPE, r2r::QosProfile::default())
    {
        Ok(client) => match Node::is_available(&client) {
            Ok(service_available) => {
                let result =
                    spawner.spawn_local(call_set_mode(service_name, service_available, client));
                if result.is_err() {
                    error!("Failed to spawn mode setup task for {service_name}.");
                }
            }
            Err(err) => warn!("Failed to monitor service {service_name}: {err:?}"),
        },
        Err(err) => {
            warn!(
                "Mode service client {service_name} is unavailable: {err:?}. \
Make sure rm_interfaces is sourced before building so r2r can generate SetMode."
            );
        }
    }
}

#[macro_export]
macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform(
                Transform::IDENTITY
                    .with_translation($translation)
                    .with_rotation($rotation),
            ),
        });
    };
    ($ls:ident, $hdr:expr, $id:expr, $transform:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform($transform),
        });
    };
}

#[macro_export]
macro_rules! pose {
    ($hdr:expr) => {
        PoseStamped {
            header: $hdr.clone(),
            pose: Pose {
                position: r2r::geometry_msgs::msg::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    };
}

/// 真值 UDP 发送器 (Resource)。真值不再走 ROS 话题, 改为 UDP JSON 直发 Linux 的 PlotJuggler。
#[derive(Resource)]
pub struct GtUdp(pub GtUdpSender);

/// 求网格所有顶点在世界系下的质心。
///
/// 为什么不能用 `GlobalTransform.translation`:
/// `vehicle.glb` 里 `ARMOR_n_P` 节点相对 BASE 的**净变换是单位阵**
/// (例如 `ARMOR_3_P` 的 `-0.2095` 恰好被父节点 `AM_3` 的 `+0.2095` 抵消, 1/2/4 号同理),
/// 节点原点永远落在旋转轴上 (= 车体中心), 四块装甲板会报成同一个点且不随旋转变化。
/// 真实的装甲板偏移烘在 mesh 顶点里, 所以必须走顶点。
///
/// 每块装甲板 mesh 只有 4 个顶点, 4 块合计 16 个, 调用成本可忽略。
/// `pub(crate)`: 命中计数 (main.rs::count_infantry_hit) 也要用它算板心。
pub(crate) fn mesh_world_centroid(gt: &GlobalTransform, mesh: &Mesh) -> Option<Vec3> {
    let mut sum = Vec3::ZERO;
    let mut count = 0usize;
    for (_attr, values) in mesh.attributes() {
        if let VertexAttributeValues::Float32x3(vec) = values {
            for &p in vec {
                sum += gt.transform_point(Vec3::from(p));
                count += 1;
            }
            break;
        }
    }
    (count > 0).then(|| sum / count as f32)
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<CaptureCamera>>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    // 敌方装甲板真值: 用于跨机真值链路 (/tf 中发布 enemy_armor_* 帧)
    // 注意: 位置必须由 mesh 顶点质心求得, 不能用节点原点 (见 mesh_world_centroid 注释)
    enemy_armors: Query<(&GlobalTransform, &Mesh3d, &Armor), With<EnemyArmor>>,
    armor_meshes: Res<Assets<Mesh>>,

    // 敌方步兵变速小陀螺状态 (仅敌方实体持有 SpinState)
    // 同时取该实体的 GlobalTransform 作为车体中心 (SpinState 与 InfantryRoot 挂在同一实体上)
    enemy_spin: Query<(&SpinState, &GlobalTransform)>,
    spin_cfg: Res<SpinConfig>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
    // 真值 UDP 发送器 (替代原先的 ROS 真值话题)
    gt_udp: Res<GtUdp>,
) {
    let cam_transform = camera.into_inner();
    let gimbal_transform = gimbal.into_inner();
    let gimbal_pose = gimbal_transform.compute_transform();
    let camera_pose = cam_transform.compute_transform();
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    let mut transform_stamped = vec![];
    let root_hdr = Header {
        stamp: stamp.clone(),
        frame_id: ROOT_FRAME_ID.to_string(),
    };
    let gimbal_hdr = Header {
        stamp: stamp.clone(),
        frame_id: GIMBAL_FRAME_ID.to_string(),
    };
    let camera_hdr = Header {
        stamp: stamp.clone(),
        frame_id: CAMERA_LINK_FRAME_ID.to_string(),
    };

    gimbal_pose_pub.publish(PoseStamped {
        header: root_hdr.clone(),
        pose: pose_from_transform(gimbal_pose),
    });
    odom_pose_pub.publish(PoseStamped {
        header: root_hdr.clone(),
        pose: pose_from_transform(Transform::IDENTITY),
    });
    camera_pose_pub.publish(PoseStamped {
        header: root_hdr.clone(),
        pose: pose_from_transform(camera_pose),
    });

    add_tf_frame!(
        transform_stamped,
        root_hdr.clone(),
        GIMBAL_FRAME_ID,
        gimbal_pose
    );
    let cam_rel = cam_transform.reparented_to(gimbal_transform);
    add_tf_frame!(
        transform_stamped,
        gimbal_hdr.clone(),
        CAMERA_LINK_FRAME_ID,
        cam_rel.translation,
        cam_rel.rotation
    );
    add_tf_frame!(
        transform_stamped,
        camera_hdr.clone(),
        CAMERA_OPTICAL_FRAME_ID,
        Vec3::ZERO,
        Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)
    );
    for (transform, rune) in runes {
        add_tf_frame!(
            transform_stamped,
            root_hdr.clone(),
            format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform.compute_transform()
        );
    }
    for (target_transform, target, name) in targets {
        if !name.contains("_ACTIVATED") {
            continue;
        }
        if let Ok((_rune_transform, rune)) = runes.get(target.1) {
            add_tf_frame!(
                transform_stamped,
                Header {
                    stamp: stamp.clone(),
                    frame_id: format!("power_rune_{:?}", rune.mode)
                        .to_string()
                        .to_lowercase(),
                },
                format!("power_rune_{:?}_{:?}", rune.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                target_transform.reparented_to(_rune_transform)
            );
        }
    }

    // 敌方装甲板真值 -> /tf, 供 Linux 小主机订阅 (enemy_armor_<NAME> 帧)
    // 平移取 mesh 顶点世界质心; 姿态沿用装甲板节点的世界旋转。
    for (armor_transform, armor_mesh, Armor(armor_name)) in &enemy_armors {
        let Some(center) = armor_meshes
            .get(armor_mesh)
            .and_then(|m| mesh_world_centroid(armor_transform, m))
        else {
            continue;
        };
        add_tf_frame!(
            transform_stamped,
            root_hdr.clone(),
            format!("enemy_armor_{}", armor_name).to_lowercase(),
            center,
            armor_transform.rotation()
        );
    }

    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });

    // ===== 真值 UDP 发送 (替代原先的 ROS /gt 话题) =====
    // ROS 系 (odom_vision, Z-up): yaw 绕 Z, pitch 绕 Y
    let gimbal_ros_rot = ros_rotation_from_bevy(gimbal_pose.rotation);
    let (gimbal_yaw, gimbal_pitch, _) = gimbal_ros_rot.to_euler(EulerRot::ZYX);

    // 云台位置 (ROS 系)
    let gimbal_pos_ros = M_ALIGN_MAT3 * gimbal_pose.translation;

    // 4 块装甲板位置: 按名称 ARMOR_1_P ~ ARMOR_4_P 固定顺序, 缺位为 None
    // 位置 = 该装甲板 mesh 顶点的世界质心 (节点原点不可用, 见 mesh_world_centroid)
    let mut armors: [Option<[f64; 3]>; 4] = [None, None, None, None];
    for idx in 1..=4u32 {
        let want = format!("armor_{}_p", idx);
        let found = enemy_armors.iter().find_map(|(gt, mesh, Armor(name))| {
            if name.to_lowercase() == want {
                armor_meshes
                    .get(mesh)
                    .and_then(|m| mesh_world_centroid(gt, m))
            } else {
                None
            }
        });
        if let Some(p) = found {
            let p = M_ALIGN_MAT3 * p;
            armors[(idx - 1) as usize] = Some([p.x as f64, p.y as f64, p.z as f64]);
        }
    }

    // 敌方步兵变速小陀螺真值 + 车体中心 (取第一个持有 SpinState 的敌方实体)
    // 注: angle 为 Bevy 绕 +Y 转角, 经 M_ALIGN_MAT3 对齐后即 ROS 系绕 +Z 的 yaw, 二者同号。
    let (spin_gt, enemy_center) = match enemy_spin.iter().next() {
        Some((s, gt)) => {
            let c = M_ALIGN_MAT3 * gt.compute_transform().translation;
            (
                Some(SpinGt {
                    mode: spin_cfg.mode.as_u8(),
                    angle: s.angle,
                    omega: s.omega,
                    alpha: s.alpha_now,
                    target: s.target,
                    phase: s.phase.as_u8(),
                    event_t: s.event_t,
                    t_acc: spin_cfg.t_acc,
                }),
                Some([c.x as f64, c.y as f64, c.z as f64]),
            )
        }
        None => (None, None),
    };

    let stamp_sec = stamp.sec as f64 + stamp.nanosec as f64 * 1e-9;
    gt_udp.0.send(
        stamp_sec,
        gimbal_yaw as f64,
        gimbal_pitch as f64,
        [gimbal_pos_ros.x as f64, gimbal_pos_ros.y as f64, gimbal_pos_ros.z as f64],
        enemy_center,
        &armors,
        spin_gt,
    );
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

/// 读一个分辨率维度环境变量, 非法值回退默认 (合法范围 64..=4096)。
fn env_dim(key: &str, default: u32) -> u32 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.trim().parse::<u32>().ok())
        .filter(|v| (64..=4096).contains(v))
        .unwrap_or(default)
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));
        let auto_aim = AutoAimBridge::default();

        publisher!(
            signal_arc,
            app,
            node,
            GlobalTransformTopic,
            GimbalPoseTopic,
            OdomPoseTopic,
            CameraPoseTopic
        );
        let camera_info = Arc::new(publisher!(signal_arc, node, CameraInfoTopic));
        let image_raw = Arc::new(publisher!(signal_arc, node, ImageRawTopic));
        let image_compressed = Arc::new(publisher!(signal_arc, node, ImageCompressedTopic));

        // 图像话题发布开关: IMAGE_PUB = raw | compressed | both | none  (默认 compressed)
        let (image_pub_raw, image_pub_compressed) = {
            let v = std::env::var("IMAGE_PUB").unwrap_or_else(|_| "compressed".to_string());
            let v = v.trim().to_ascii_lowercase();
            let raw = matches!(v.as_str(), "raw" | "both");
            let compressed = matches!(v.as_str(), "compressed" | "both");
            if !raw && !compressed && v != "none" {
                warn!(
                    "IMAGE_PUB=\"{v}\" 无法识别 (应为 raw/compressed/both/none), 按 compressed 处理"
                );
                (false, true)
            } else {
                (raw, compressed)
            }
        };
        info!(
            "[IMAGE] IMAGE_PUB => publish_raw={image_pub_raw} publish_compressed={image_pub_compressed} jpeg_quality={}",
            crate::util::image::jpeg_quality()
        );

        // 采集分辨率: CAPTURE_WIDTH / CAPTURE_HEIGHT (默认 960x720)
        // 内参由 capture.rs::compute_camera 从 fov_y + 分辨率重算 (f_x == f_y 恒成立),
        // 所以改分辨率后自瞄从 /camera_info 读到的新内参可以直接用。
        let capture_width = env_dim("CAPTURE_WIDTH", 960);
        let capture_height = env_dim("CAPTURE_HEIGHT", 720);
        info!(
            "[IMAGE] capture resolution => {capture_width}x{capture_height} (CAPTURE_WIDTH / CAPTURE_HEIGHT)"
        );

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());
        let shotlog = crate::shotlog::ShotLog::from_env();

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(auto_aim.clone())
            .insert_resource(shotlog.clone())
            .insert_resource(GtUdp(GtUdpSender::from_env()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .add_plugins(RosCapturePlugin {
                config: CaptureConfig {
                    width: capture_width,
                    height: capture_height,
                    texture_format: TextureFormat::bevy_default(),
                    fov_y: PI / 180.0 * 45.0,
                },
                context: RosCaptureContext {
                    clock,
                    camera_info,
                    image_raw,
                    image_compressed,
                    publish_raw: image_pub_raw,
                    publish_compressed: image_pub_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(Update, capture_rune.after(TransformSystems::Propagate))
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                let mut pool = LocalPool::new();
                let spawner = pool.spawner();
                setup_auto_aim_subscriptions(&mut node, &spawner, auto_aim, shotlog);
                setup_mode_service_client(&mut node, &spawner, RUNE_DETECTOR_SET_MODE_SERVICE);
                setup_mode_service_client(&mut node, &spawner, RUNE_SOLVER_SET_MODE_SERVICE);

                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1000));
                    pool.run_until_stalled();
                }
            }))));
    }
}
