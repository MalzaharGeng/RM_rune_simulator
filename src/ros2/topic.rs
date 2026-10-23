use bevy::prelude::Resource;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::sensor_msgs::msg::{CameraInfo, CompressedImage, Image};
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{QosProfile, WrappedTypesupport};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::SyncSender;

// ─────────────────────────────────────────────────────────────────────────────
// [CAM] 发送侧失败计数 (2026-10-13 加入, 见 ros2/capture.rs 顶部注释)
//   这两处失败原来都是**静默**的:
//     1. TopicPublisher::publish 用 try_send, 通道(1024)满了直接丢帧
//     2. publisher 线程里 publisher.publish() 报错时, while !sent 循环丢队列再重试
//   出图率异常低时, 靠这两个计数区分「采集没出图」和「出了图发不出去」。
// ─────────────────────────────────────────────────────────────────────────────
// `publisher!` 是 #[macro_export]，会在调用点(crate 内)展开成
// `crate::ros2::topic::PUB_WRITE_ERR`，所以这两个计数必须是 pub。
pub static PUB_WRITE_ERR: AtomicU64 = AtomicU64::new(0);
pub static PUB_TRY_SEND_ERR: AtomicU64 = AtomicU64::new(0);

/// 取出并清零自上次调用以来的发送失败数（供 [CAM] 日志用）。
pub fn take_publish_errors() -> u64 {
    PUB_WRITE_ERR.swap(0, Ordering::Relaxed) + PUB_TRY_SEND_ERR.swap(0, Ordering::Relaxed)
}

#[derive(Resource)]
pub struct TopicPublisher<T: RosTopic> {
    sender: SyncSender<T::T>,
}

impl<T: RosTopic> TopicPublisher<T> {
    pub(crate) fn new(sender: SyncSender<T::T>) -> Self {
        TopicPublisher { sender }
    }

    pub fn publish(&self, message: T::T) {
        if self.sender.try_send(message).is_err() {
            PUB_TRY_SEND_ERR.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[macro_export]
macro_rules! publisher {
    ($node:ident,$topic:ty) => {
        {
            let (sender, receiver): (
                ::std::sync::mpsc::SyncSender<<$topic as crate::ros2::topic::RosTopic>::T>,
                ::std::sync::mpsc::Receiver<<$topic as crate::ros2::topic::RosTopic>::T>,
            ) = ::std::sync::mpsc::sync_channel(1024);

            let publisher = $node.create_publisher(
                <$topic>::TOPIC,
                <$topic>::QOS,
            ).unwrap();

            (receiver, sender, publisher)
        }
    };
    ($atomic:expr, $node:ident, $topic:ty) => {{
        let atomic = $atomic.clone();
        let (receiver,sender,publisher) = publisher!($node, $topic);
        ::std::thread::spawn(move || {
            while !atomic.load(::std::sync::atomic::Ordering::Acquire) {
                let mut did_work = false;
                loop {
                    match receiver.recv_timeout(Duration::from_secs(1)) {
                        Ok(m) => {
                            let mut sent = false;
                            while !sent {
                                match publisher.publish(&m) {
                                    Ok(_) => sent = true,
                                    Err(_) => {
                                        crate::ros2::topic::PUB_WRITE_ERR
                                            .fetch_add(1, ::std::sync::atomic::Ordering::Relaxed);
                                        let _ = receiver.try_recv();
                                    }
                                }
                            }
                            did_work = true;
                        }
                        Err(::std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(::std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
                    }
                }
                if !did_work {
                    ::std::thread::sleep(::std::time::Duration::from_millis(1));
                }
            }
        });
        crate::ros2::topic::TopicPublisher::<$topic>::new(sender)
    }};

    ($atomic:expr, $app:ident, $node:ident, $($topic:ty),* $(,)?) => {
        $(
            $app.insert_resource(publisher!($atomic, $node, $topic));
        )*
    };
}

pub trait RosTopic {
    type T: WrappedTypesupport + 'static;
    const TOPIC: &'static str;
    const QOS: QosProfile;
}

macro_rules! define_topic {
    ($topic:ident, $typ:ty, $url:expr, $qos:expr) => {
        pub struct $topic;
        impl RosTopic for $topic {
            type T = $typ;
            const TOPIC: &'static str = $url;
            const QOS: QosProfile = $qos;
        }
    };
    ($topic:ident, $typ:ty, $url:expr) => {
        define_topic!($topic, $typ, $url, ::r2r::QosProfile::default());
    };
}

define_topic!(CameraInfoTopic, CameraInfo, "/camera_info");
define_topic!(ImageRawTopic, Image, "/image_raw");
define_topic!(ImageCompressedTopic, CompressedImage, "/image_compressed");
define_topic!(GlobalTransformTopic, TFMessage, "/tf");

define_topic!(GimbalPoseTopic, PoseStamped, "/gimbal_pose");
define_topic!(OdomPoseTopic, PoseStamped, "/odom_pose");
define_topic!(CameraPoseTopic, PoseStamped, "/camera_pose");
