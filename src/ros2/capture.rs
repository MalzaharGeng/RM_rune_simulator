use crate::{InfantryGimbal, InfantryViewOffset, LocalInfantry};
use crate::ros2::topic::{CameraInfoTopic, ImageCompressedTopic, ImageRawTopic, TopicPublisher};
use crate::util::image::compress_image;
use bevy::anti_alias::fxaa::Fxaa;
use bevy::app::PostUpdate;
use bevy::transform::TransformSystems;
use bevy::camera::Exposure;
use bevy::camera::visibility::RenderLayers;
use bevy::post_process::bloom::Bloom;
use bevy::render::view::Hdr;
use bevy::tasks::AsyncComputeTaskPool;
use bevy::{
    image::TextureFormatPixelInfo,
    prelude::*,
    render::{
        Extract, Render, RenderApp, RenderSystems,
        render_asset::RenderAssets,
        render_graph::{self, NodeRunError, RenderGraph, RenderGraphContext, RenderLabel},
        render_resource::{
            Buffer, BufferDescriptor, BufferUsages, CommandEncoderDescriptor, Extent3d, MapMode,
            PollType, TexelCopyBufferInfo, TexelCopyBufferLayout, TextureFormat, TextureUsages,
        },
        renderer::{RenderContext, RenderDevice, RenderQueue},
    },
};
use r2r::Clock;
use r2r::sensor_msgs::msg::{CameraInfo, RegionOfInterest};
use r2r::std_msgs::msg::Header;
use std::sync::{
    Arc, Mutex, OnceLock,
    atomic::{AtomicU64, AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

// ─────────────────────────────────────────────────────────────────────────────
// [CAM] 采集链路诊断计数器 (2026-10-13 加入)
//
// 背景: 自瞄侧实测只收到 ~0.6 帧/s, 而 [SIM_GT] 显示仿真本体跑 140+ fps、
//   RateLimiter 上限 60 Hz ⇒ 必须区分「仿真没在出图」和「出了图但发不出去」。
//   TopicPublisher::publish 用的是 try_send, 通道满了会**静默丢帧**;
//   publisher 线程在 publish() 报错时也是 while !sent 里静默丢。
//   所以这里同时统计: 采集任务数 / poll(Wait) 耗时 / 任务总耗时 / DDS 发送失败数。
//
// 输出: 每 ~2 s 一行 [CAM] n=<窗口内任务数> hz=<> poll=<>ms task=<>ms send_err=<>
//   send_err 是 topic.rs 里 PUB_WRITE_ERR + PUB_TRY_SEND_ERR 的增量。
// ─────────────────────────────────────────────────────────────────────────────
static CAM_TASKS: AtomicU64 = AtomicU64::new(0);
static CAM_POLL_US: AtomicU64 = AtomicU64::new(0);
static CAM_TASK_US: AtomicU64 = AtomicU64::new(0);
static CAM_SKIPPED: AtomicU64 = AtomicU64::new(0);
static CAM_EPOCH: OnceLock<Instant> = OnceLock::new();
static CAM_LAST_LOG_MS: AtomicU64 = AtomicU64::new(0);

/// 每个采集任务结束时调用一次；每 ~2 s 打一行 [CAM]。
fn cam_stat_log(poll_us: u64, task_us: u64) {
    let epoch = *CAM_EPOCH.get_or_init(Instant::now);
    let n = CAM_TASKS.fetch_add(1, Ordering::Relaxed) + 1;
    CAM_POLL_US.fetch_add(poll_us, Ordering::Relaxed);
    CAM_TASK_US.fetch_add(task_us, Ordering::Relaxed);

    let now_ms = epoch.elapsed().as_millis() as u64;
    let last = CAM_LAST_LOG_MS.load(Ordering::Relaxed);
    if now_ms.saturating_sub(last) < 2000 {
        return;
    }
    if CAM_LAST_LOG_MS
        .compare_exchange(last, now_ms, Ordering::Relaxed, Ordering::Relaxed)
        .is_err()
    {
        return; // 另一个任务已经打过这一轮
    }
    let win_ms = now_ms.saturating_sub(last).max(1);
    let inv = 1.0 / n as f64;
    let send_err = crate::ros2::topic::take_publish_errors();
    info!(
        "[CAM] n={} hz={:.2} poll={:.2}ms task={:.2}ms skipped={} send_err={}",
        n,
        n as f64 * 1000.0 / win_ms as f64,
        CAM_POLL_US.swap(0, Ordering::Relaxed) as f64 * inv / 1000.0,
        CAM_TASK_US.swap(0, Ordering::Relaxed) as f64 * inv / 1000.0,
        CAM_SKIPPED.swap(0, Ordering::Relaxed),
        send_err,
    );
    CAM_TASKS.store(0, Ordering::Relaxed);
}

/// 采集任务被 `copying` 门挡掉的次数（说明回读还没回来，主循环来早了）。
pub(crate) fn cam_stat_skipped() {
    CAM_SKIPPED.fetch_add(1, Ordering::Relaxed);
}

#[derive(Resource, Clone)]
pub struct CaptureConfig {
    pub width: u32,
    pub height: u32,
    pub texture_format: TextureFormat,
    pub fov_y: f32,
}

#[derive(Clone, Default, Resource, Deref, DerefMut)]
struct ImageCopiers(pub Vec<ImageCopier>);

#[derive(Clone, Component)]
struct ImageCopier {
    buffer: Buffer,
    copying: Arc<AtomicBool>,
    enabled: Arc<AtomicBool>,
    src_image: Handle<Image>,
}

impl ImageCopier {
    pub fn new(
        src_image: Handle<Image>,
        size: Extent3d,
        render_device: &RenderDevice,
    ) -> ImageCopier {
        let padded_bytes_per_row = RenderDevice::align_copy_bytes_per_row(size.width as usize) * 4;

        let cpu_buffer = render_device.create_buffer(&BufferDescriptor {
            label: None,
            size: padded_bytes_per_row as u64 * size.height as u64,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        ImageCopier {
            buffer: cpu_buffer,
            src_image,
            copying: Arc::new(AtomicBool::new(false)),
            enabled: Arc::new(AtomicBool::new(true)),
        }
    }

    pub fn enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }
}

fn image_copy_extract(mut commands: Commands, image_copiers: Extract<Query<&ImageCopier>>) {
    commands.insert_resource(ImageCopiers(
        image_copiers.iter().cloned().collect::<Vec<ImageCopier>>(),
    ));
}

#[derive(Debug, PartialEq, Eq, Clone, Hash, RenderLabel)]
struct ImageCopy;

#[derive(Default)]
struct ImageCopyDriver;

impl render_graph::Node for ImageCopyDriver {
    fn run(
        &self,
        _graph: &mut RenderGraphContext,
        render_context: &mut RenderContext,
        world: &World,
    ) -> Result<(), NodeRunError> {
        let image_copiers = world.get_resource::<ImageCopiers>().unwrap();
        let gpu_images = world
            .get_resource::<RenderAssets<bevy::render::texture::GpuImage>>()
            .unwrap();

        for image_copier in image_copiers.iter() {
            if !image_copier.enabled() || image_copier.copying.load(Ordering::Acquire) {
                continue;
            }
            let src_image = gpu_images.get(&image_copier.src_image).unwrap();
            let mut encoder = render_context
                .render_device()
                .create_command_encoder(&CommandEncoderDescriptor::default());
            let block_dimensions = src_image.texture_format.block_dimensions();
            let block_size = src_image.texture_format.block_copy_size(None).unwrap();
            let padded_bytes_per_row = RenderDevice::align_copy_bytes_per_row(
                (src_image.size.width as usize / block_dimensions.0 as usize) * block_size as usize,
            );

            encoder.copy_texture_to_buffer(
                src_image.texture.as_image_copy(),
                TexelCopyBufferInfo {
                    buffer: &image_copier.buffer,
                    layout: TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(
                            std::num::NonZero::<u32>::new(padded_bytes_per_row as u32)
                                .unwrap()
                                .into(),
                        ),
                        rows_per_image: None,
                    },
                },
                src_image.size,
            );
            let render_queue = world.get_resource::<RenderQueue>().unwrap();
            render_queue.submit(std::iter::once(encoder.finish()));
        }
        Ok(())
    }
}

fn receive_image_from_buffer(
    t: Res<Time>,
    mut r: ResMut<RateLimiter>,
    image_copiers: Res<ImageCopiers>,
    render_device: Res<RenderDevice>,
    config: Res<CaptureConfig>,
    ctx: Res<RosCaptureContext>,
) {
    r.tick(t.delta());
    if !r.is_finished() {
        return;
    }
    r.reset();
    let (width, height, texture_format) = (config.width, config.height, config.texture_format);
    for image_copier in image_copiers.0.iter() {
        if !image_copier.enabled() {
            continue;
        }

        if image_copier
            .copying
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
        {
            let render_device = render_device.clone();
            let image_copier = image_copier.clone();
            let ctx = (*ctx).clone();
            let config = (*config).clone();
            AsyncComputeTaskPool::get()
                .spawn(async move {
                    let t_task = Instant::now();
                    let buffer_slice = image_copier.buffer.slice(..);
                    let (s, r) = crossbeam_channel::bounded(1);

                    buffer_slice.map_async(MapMode::Read, move |r| match r {
                        Ok(r) => s.send(r).expect("Failed to send map update"),
                        Err(err) => panic!("Failed to map buffer {err}"),
                    });

                    render_device
                        .poll(PollType::Wait)
                        .expect("Failed to poll device for map async");
                    r.recv().expect("Failed to receive the map_async message");
                    let poll_us = t_task.elapsed().as_micros() as u64;

                    let mut image_data = buffer_slice.get_mapped_range().to_vec();
                    image_copier.buffer.unmap();
                    image_copier.copying.store(false, Ordering::Release);

                    let mut image_data = {
                        let row_bytes = width as usize * texture_format.pixel_size().unwrap();
                        let aligned_row_bytes = RenderDevice::align_copy_bytes_per_row(row_bytes);
                        if row_bytes != aligned_row_bytes {
                            image_data = image_data
                                .chunks(aligned_row_bytes)
                                .take(height as usize)
                                .flat_map(|row| &row[..row_bytes.min(row.len())])
                                .cloned()
                                .collect();
                        }
                        let mut bevy_image =
                            Image::new_target_texture(width, height, texture_format);
                        bevy_image.data = Some(image_data);
                        bevy_image.try_into_dynamic().unwrap().to_rgb8().into_raw()
                    };
                    let optical_frame_hdr = Header {
                        stamp: Clock::to_builtin_time(
                            &ctx.clock.lock().unwrap().get_now().unwrap(),
                        ),
                        frame_id: "camera_optical_frame".to_string(),
                    };
                    // 图像话题发布由环境变量 IMAGE_PUB 控制 (raw / compressed / both / none)
                    if ctx.publish_compressed {
                        // Only both topics need separate ownership of the RGB pixels.
                        let compressed_pixels = if ctx.publish_raw {
                            image_data.clone()
                        } else {
                            std::mem::take(&mut image_data)
                        };
                        ctx.image_compressed.publish(compress_image(
                            optical_frame_hdr.clone(),
                            width,
                            height,
                            compressed_pixels,
                        ));
                    }
                    let (camera_info, image) = compute_camera(
                        config.fov_y,
                        optical_frame_hdr,
                        config.width,
                        config.height,
                        image_data,
                    );
                    ctx.camera_info.publish(camera_info);
                    if ctx.publish_raw {
                        // 原始 Image (rgb8, 960x720, step=2880) —— 不压缩, 带宽 ~2 MB/帧
                        ctx.image_raw.publish(image);
                    }
                    cam_stat_log(poll_us, t_task.elapsed().as_micros() as u64);
                })
                .detach();
        } else {
            // copying 门被占用 ⇒ 这一帧没有发起回读 (主循环比回读还快)
            cam_stat_skipped();
        }
    }
}

#[derive(Resource, Clone)]
pub struct RosCaptureContext {
    pub clock: Arc<Mutex<Clock>>,
    pub camera_info: Arc<TopicPublisher<CameraInfoTopic>>,
    pub image_raw: Arc<TopicPublisher<ImageRawTopic>>,
    pub image_compressed: Arc<TopicPublisher<ImageCompressedTopic>>,
    /// 是否发布 /image_raw（原始 Image）
    pub publish_raw: bool,
    /// 是否发布 /image_compressed（JPEG 压缩）
    pub publish_compressed: bool,
}

pub struct RosCapturePlugin {
    pub config: CaptureConfig,
    pub context: RosCaptureContext,
}

#[derive(Resource, Deref, DerefMut)]
struct RateLimiter(Timer);

impl Plugin for RosCapturePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone())
            .add_systems(PostUpdate, sync_camera.after(TransformSystems::Propagate))
            .add_systems(Startup, setup_capture_scene);
        let render_app = app.sub_app_mut(RenderApp);

        let mut graph = render_app.world_mut().resource_mut::<RenderGraph>();
        graph.add_node(ImageCopy, ImageCopyDriver);
        graph.add_node_edge(bevy::render::graph::CameraDriverLabel, ImageCopy);

        render_app
            .insert_resource(self.config.clone())
            .insert_resource(self.context.clone())
            .add_systems(ExtractSchedule, image_copy_extract)
            .insert_resource(RateLimiter(Timer::new(
                Duration::from_secs_f64(1.0 / 60.0),
                TimerMode::Once,
            )))
            .add_systems(
                Render,
                receive_image_from_buffer.after(RenderSystems::Render),
            );
    }
}

#[derive(Component)]
pub struct CaptureCamera;

fn setup_capture_scene(
    mut commands: Commands,
    mut images: ResMut<Assets<Image>>,
    render_device: Res<RenderDevice>,
    config: Res<CaptureConfig>,
) {
    let size = Extent3d {
        width: config.width,
        height: config.height,
        ..Default::default()
    };
    let mut render_target_image =
        Image::new_target_texture(size.width, size.height, config.texture_format);
    render_target_image.texture_descriptor.usage |= TextureUsages::COPY_SRC;
    let render_target_handle = images.add(render_target_image);

    commands.spawn(ImageCopier::new(
        render_target_handle.clone(),
        size,
        &render_device,
    ));

    commands.spawn((
        Camera3d::default(),
        Bloom::NATURAL,
        Camera {
            target: render_target_handle.clone().into(),
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov: config.fov_y,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Exposure::SUNLIGHT,
        Msaa::Off,
        Fxaa::default(),
        RenderLayers::layer(0),
        Hdr,
        CaptureCamera,
    ));
}

fn sync_camera(
    gimbal: Single<
        (&GlobalTransform, &Transform),
        (With<LocalInfantry>, With<InfantryGimbal>, Without<CaptureCamera>),
    >,
    view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
    mut our: Single<&mut Transform, (With<CaptureCamera>, Without<InfantryGimbal>)>,
    mut trace: Local<Option<bool>>,
) {
    let (gimbal_global, gimbal_local) = gimbal.into_inner();

    // 原来这里逐帧 println!，会把 stdout 刷爆 (实测 8.5 小时写了 108 MB)，也拖慢主循环。
    // 改成默认关闭，需要时用 SYNC_CAMERA_LOG=1 打开。
    if *trace.get_or_insert_with(|| {
        std::env::var("SYNC_CAMERA_LOG")
            .map(|v| v.trim() == "1")
            .unwrap_or(false)
    }) {
        let (local_yaw, local_pitch, _) =
            gimbal_local.rotation.to_euler(EulerRot::YXZ);
        let (global_yaw, global_pitch, _) =
            gimbal_global.rotation().to_euler(EulerRot::YXZ);

        println!(
            "[SYNC_CAMERA] local_yaw={:.3} local_pitch={:.3} global_yaw={:.3} global_pitch={:.3} dyaw={:.3} dpitch={:.3}",
            local_yaw.to_degrees(),
            local_pitch.to_degrees(),
            global_yaw.to_degrees(),
            global_pitch.to_degrees(),
            (local_yaw - global_yaw).to_degrees(),
            (local_pitch - global_pitch).to_degrees(),
        );
    }

    let camera_offset = view_offset.0.translation - gimbal_local.translation;
    our.translation =
        gimbal_global.translation() + gimbal_global.rotation() * camera_offset;
    our.scale = Vec3::ONE;
    // Bevy's camera looks along -Z, which is ROS gimbal +X after M_ALIGN_MAT3.
    our.rotation = gimbal_global.rotation();
}

fn compute_camera(
    fov_y: f32,
    hdr: Header,
    width: u32,
    height: u32,
    data: Vec<u8>,
) -> (CameraInfo, r2r::sensor_msgs::msg::Image) {
    let fov_y = fov_y as f64;
    let (width, height) = (width, height);

    let (fov_y, fov_x) = {
        let aspect = width as f64 / height as f64;
        let fov_x = 2.0 * ((fov_y / 2.0).tan() * aspect).atan();
        (fov_y, fov_x)
    };

    let f_x = width as f64 / (2.0 * (fov_x / 2.0).tan());
    let f_y = height as f64 / (2.0 * (fov_y / 2.0).tan());

    let c_x = width as f64 / 2.0;
    let c_y = height as f64 / 2.0;

    // Removed x-axis flip; rely on optical rotation instead

    (
        CameraInfo {
            header: hdr.clone(),
            height,
            width,
            distortion_model: "plumb_bob".to_string(),
            d: vec![0.000, 0.000, 0.000, 0.000, 0.000],
            k: vec![f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0],
            p: vec![f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0],
            r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest {
                x_offset: 0,
                y_offset: 0,
                height,
                width,
                do_rectify: true,
            },
        },
        r2r::sensor_msgs::msg::Image {
            header: hdr,
            height,
            width,
            encoding: "rgb8".to_string(),
            is_bigendian: 0,
            step: width * 3,
            data,
        },
    )
}
