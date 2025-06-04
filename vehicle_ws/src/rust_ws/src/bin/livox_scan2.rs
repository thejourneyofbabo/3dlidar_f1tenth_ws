use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::PointCloud2;
use std::env;

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

impl LidarPoint {
    fn from_bytes(data: &[u8], offset: usize) -> Option<Self> {
        if offset + 26 > data.len() {
            return None;
        }

        // 리틀 엔디안으로 바이트를 변환
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

fn print_point_cloud_summary(msg: &PointCloud2) {
    let points = parse_pointcloud2(msg);

    println!("=== LiDAR Point Cloud Data ===");
    println!("Frame ID: {}", msg.header.frame_id);
    println!(
        "Timestamp: {}.{:09}",
        msg.header.stamp.sec, msg.header.stamp.nanosec
    );
    println!("Total Points: {}", points.len());
    println!("Point Step: {} bytes", msg.point_step);
    println!();

    if !points.is_empty() {
        // 첫 5개 포인트 상세 출력
        println!("First 5 Points:");
        println!(
            "{:<6} {:<10} {:<10} {:<10} {:<10} {:<4} {:<4} {:<15}",
            "Index", "X(m)", "Y(m)", "Z(m)", "Intensity", "Tag", "Line", "Timestamp"
        );
        println!("{}", "-".repeat(80));

        for (i, point) in points.iter().take(5).enumerate() {
            println!(
                "{:<6} {:<10.3} {:<10.3} {:<10.3} {:<10.1} {:<4} {:<4} {:<15.3}",
                i,
                point.x,
                point.y,
                point.z,
                point.intensity,
                point.tag,
                point.line,
                point.timestamp
            );
        }

        if points.len() > 5 {
            println!("... and {} more points", points.len() - 5);
        }

        // 통계 정보
        let x_values: Vec<f32> = points.iter().map(|p| p.x).collect();
        let y_values: Vec<f32> = points.iter().map(|p| p.y).collect();
        let z_values: Vec<f32> = points.iter().map(|p| p.z).collect();
        let intensity_values: Vec<f32> = points.iter().map(|p| p.intensity).collect();

        println!("\n=== Statistics ===");
        println!(
            "X range: {:.3} ~ {:.3} m",
            x_values.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
            x_values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b))
        );
        println!(
            "Y range: {:.3} ~ {:.3} m",
            y_values.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
            y_values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b))
        );
        println!(
            "Z range: {:.3} ~ {:.3} m",
            z_values.iter().fold(f32::INFINITY, |a, &b| a.min(b)),
            z_values.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b))
        );
        println!(
            "Intensity range: {:.1} ~ {:.1}",
            intensity_values
                .iter()
                .fold(f32::INFINITY, |a, &b| a.min(b)),
            intensity_values
                .iter()
                .fold(f32::NEG_INFINITY, |a, &b| a.max(b))
        );

        // 라인별 포인트 개수
        let mut line_counts = std::collections::HashMap::new();
        for point in &points {
            *line_counts.entry(point.line).or_insert(0) += 1;
        }

        println!("\n=== Points per Line ===");
        let mut lines: Vec<_> = line_counts.iter().collect();
        lines.sort_by_key(|&(line, _)| line);
        for (line, count) in lines.iter().take(10) {
            println!("Line {}: {} points", line, count);
        }
        if lines.len() > 10 {
            println!("... and {} more lines", lines.len() - 10);
        }
    }

    println!("{}", "=".repeat(50));
    println!();
}

fn main() -> Result<(), Error> {
    println!("This is LiDAR Scan node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_scanner")?;
    let _subscriber = node.create_subscription::<PointCloud2, _>(
        "/livox/lidar",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PointCloud2| {
            print_point_cloud_summary(&msg);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
