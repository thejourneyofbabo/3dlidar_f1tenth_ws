use anyhow::{Error, Result};
use rclrs::{self, Context, Publisher};
use sensor_msgs::msg::PointCloud2;
use std::env;
use std::sync::Arc;
use std_msgs::msg::Header;

#[derive(Debug)]
struct LidarPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    tag: u8,
    line: u8,
    timestamp: f64,
}

#[derive(Debug)]
struct BevPoint {
    x: f32,
    y: f32,
    z: f32, // Z축 추가
    intensity: f32,
    tag: u8,
    line: u8,
    timestamp: f64,
}

impl LidarPoint {
    fn from_bytes(data: &[u8], offset: usize) -> Option<Self> {
        if offset + 26 > data.len() {
            return None;
        }

        let x = f32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        let y = f32::from_le_bytes([
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);
        let z = f32::from_le_bytes([
            data[offset + 8],
            data[offset + 9],
            data[offset + 10],
            data[offset + 11],
        ]);
        let intensity = f32::from_le_bytes([
            data[offset + 12],
            data[offset + 13],
            data[offset + 14],
            data[offset + 15],
        ]);
        let tag = data[offset + 16];
        let line = data[offset + 17];
        let timestamp = f64::from_le_bytes([
            data[offset + 18],
            data[offset + 19],
            data[offset + 20],
            data[offset + 21],
            data[offset + 22],
            data[offset + 23],
            data[offset + 24],
            data[offset + 25],
        ]);

        Some(LidarPoint {
            x,
            y,
            z,
            intensity,
            tag,
            line,
            timestamp,
        })
    }

    fn to_bev(&self) -> BevPoint {
        BevPoint {
            x: self.x,
            y: self.y,
            z: 0.0, // BEV에서는 Z=0
            intensity: self.intensity,
            tag: self.tag,
            line: self.line,
            timestamp: self.timestamp,
        }
    }
}

impl BevPoint {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(26); // Z축 포함하여 26바이트

        // X, Y, Z 좌표 (각각 4바이트)
        bytes.extend_from_slice(&self.x.to_le_bytes());
        bytes.extend_from_slice(&self.y.to_le_bytes());
        bytes.extend_from_slice(&self.z.to_le_bytes()); // Z축 추가

        // intensity (4바이트)
        bytes.extend_from_slice(&self.intensity.to_le_bytes());

        // tag, line (각각 1바이트)
        bytes.push(self.tag);
        bytes.push(self.line);

        // timestamp (8바이트)
        bytes.extend_from_slice(&self.timestamp.to_le_bytes());

        bytes
    }
}

fn parse_pointcloud2(msg: &PointCloud2) -> Vec<LidarPoint> {
    let mut points = Vec::new();
    let point_step = msg.point_step as usize;

    for i in (0..msg.data.len()).step_by(point_step) {
        if let Some(point) = LidarPoint::from_bytes(&msg.data, i) {
            points.push(point);
        }
    }

    points
}

fn create_bev_pointcloud2(points: Vec<BevPoint>, original_header: &Header) -> PointCloud2 {
    use sensor_msgs::msg::PointField;

    // BEV PointField 정의 (Z축 포함)
    let mut fields = Vec::new();

    // X 좌표
    fields.push(PointField {
        name: "x".to_string(),
        offset: 0,
        datatype: 7, // FLOAT32
        count: 1,
    });

    // Y 좌표
    fields.push(PointField {
        name: "y".to_string(),
        offset: 4,
        datatype: 7, // FLOAT32
        count: 1,
    });

    // Z 좌표 (추가)
    fields.push(PointField {
        name: "z".to_string(),
        offset: 8,
        datatype: 7, // FLOAT32
        count: 1,
    });

    // Intensity
    fields.push(PointField {
        name: "intensity".to_string(),
        offset: 12,
        datatype: 7, // FLOAT32
        count: 1,
    });

    // Tag
    fields.push(PointField {
        name: "tag".to_string(),
        offset: 16,
        datatype: 2, // UINT8
        count: 1,
    });

    // Line
    fields.push(PointField {
        name: "line".to_string(),
        offset: 17,
        datatype: 2, // UINT8
        count: 1,
    });

    // Timestamp
    fields.push(PointField {
        name: "timestamp".to_string(),
        offset: 18,
        datatype: 8, // FLOAT64
        count: 1,
    });

    // 모든 포인트의 바이트 데이터 생성
    let mut data = Vec::new();
    for point in points.iter() {
        data.extend_from_slice(&point.to_bytes());
    }

    // 새로운 헤더 생성 (frame_id를 BEV로 변경)
    let mut bev_header = original_header.clone();
    bev_header.frame_id = format!("{}_bev", original_header.frame_id);

    PointCloud2 {
        header: bev_header,
        height: 1,
        width: points.len() as u32,
        fields,
        is_bigendian: false,
        point_step: 26, // Z축 포함하여 26바이트
        row_step: (points.len() * 26) as u32,
        data,
        is_dense: true,
    }
}

fn process_and_publish_bev(
    msg: PointCloud2,
    publisher: &Arc<Publisher<PointCloud2>>,
) -> Result<(), Error> {
    // 1. 원본 3D 포인트 파싱
    let lidar_points = parse_pointcloud2(&msg);
    let original_count = lidar_points.len(); // 먼저 개수 저장

    // 2. Z축 필터링 후 BEV 포인트로 변환
    let bev_points: Vec<BevPoint> = lidar_points
        .into_iter()
        .filter(|point| point.z >= -0.1 && point.z <= 0.2) // Z축 필터링
        .map(|point| point.to_bev())
        .collect();

    println!("원본 포인트 수: {}", original_count);
    println!("필터링 후 BEV 포인트 수: {}", bev_points.len());

    // 3. 새로운 PointCloud2 메시지 생성
    let bev_msg = create_bev_pointcloud2(bev_points, &msg.header);

    // 4. BEV 토픽으로 발행
    publisher.publish(bev_msg)?;

    println!("BEV 포인트 클라우드 발행 완료!");

    Ok(())
}

fn main() -> Result<(), Error> {
    println!("LiDAR BEV Publisher Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_bev_publisher")?;

    // BEV 포인트 클라우드 발행자 생성
    let bev_publisher =
        node.create_publisher::<PointCloud2>("/livox/lidar_bev", rclrs::QOS_PROFILE_DEFAULT)?;
    let bev_publisher = Arc::new(bev_publisher);

    // 원본 LiDAR 구독자 생성
    let publisher_clone = Arc::clone(&bev_publisher);
    let _subscriber = node.create_subscription::<PointCloud2, _>(
        "/livox/lidar",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PointCloud2| {
            if let Err(e) = process_and_publish_bev(msg, &publisher_clone) {
                eprintln!("BEV 처리 중 오류: {}", e);
            }
        },
    )?;

    println!("구독 토픽: /livox/lidar");
    println!("발행 토픽: /livox/lidar_bev");
    println!("BEV 변환 시작...");

    rclrs::spin(node).map_err(|err| err.into())
}
