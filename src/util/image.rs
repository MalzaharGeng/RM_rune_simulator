use r2r::sensor_msgs::msg::CompressedImage;
use r2r::std_msgs::msg::Header;

/// JPEG 压缩质量 (1-100)，由环境变量 `JPEG_QUALITY` 控制，**默认 95**。
///
/// 历史默认是 75（`JpegEncoder::new()` 的默认值）。2026-10-11 重新实测
/// （960x720, 本机 -> 自瞄 2.5Gbps 有线, 每种质量 20 s x2）：
///
/// | 质量 | 帧大小 | 带宽 | 实测出图率 |
/// |---|---|---|---|
/// | 75  | ~20 KB  | ~8 Mbps  | (未测, 与 95 同级) |
/// | 95  | 48.8 KB | 18~21 Mbps | 54.4 / 46.8 Hz |
/// | 100 | 194.8 KB| 75 Mbps  | 47.99 / 47.81 Hz |
///
/// **结论：JPEG 质量对帧率没有可测影响。** 编码跑在 `AsyncComputeTaskPool` 里
/// 异步执行，而瓶颈是 GPU->CPU 回读（约 20 ms/帧，连 60 Hz 的 `RateLimiter` 都到不了），
/// 所以提高质量是"免费"的。轮间抖动（47~54 Hz）比质量带来的差异还大。
///
/// 另外：`image` 0.25.8 的编码器**实际输出 4:4:4**（SOF 里三个分量 h=1 v=1），
/// 源码里那句 "subsampling ratio 4:2:2" 的注释是过时的 —— 所以没有色度抽样损失，
/// q100 时量化表全为 1，已经接近无损，JPEG 这条路到头了。
/// 想要真正无损只能走 `/image_raw`（736 Mbps），但那会破坏自瞄对
/// `/image_compressed` 的订阅，不划算。
///
/// 想回退到旧行为：`set JPEG_QUALITY=75`（**不用重编**）。
pub fn jpeg_quality() -> u8 {
    static QUALITY: std::sync::OnceLock<u8> = std::sync::OnceLock::new();
    *QUALITY.get_or_init(|| {
        std::env::var("JPEG_QUALITY")
            .ok()
            .and_then(|s| s.trim().parse::<u8>().ok())
            .filter(|q| (1..=100).contains(q))
            .unwrap_or(95)
    })
}

pub fn compress_image(header: Header, width: u32, height: u32, data: Vec<u8>) -> CompressedImage {
    use image::codecs::jpeg::JpegEncoder;
    use image::{ImageBuffer, Rgb};
    use std::io::Cursor;

    let buffer: ImageBuffer<Rgb<u8>, _> =
        ImageBuffer::from_raw(width, height, data).unwrap();

    let mut cursor = Cursor::new(Vec::new());
    let mut encoder = JpegEncoder::new_with_quality(&mut cursor, jpeg_quality());
    encoder.encode_image(&buffer).expect("JPEG encode failed");
    let compressed_data = cursor.into_inner();

    CompressedImage {
        header,
        format: "jpeg".to_string(),
        data: compressed_data,
    }
}
