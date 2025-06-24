use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::PointCloud2;

/// Represents a 3D LiDAR point with all sensor data
#[derive(Debug, Clone)]
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
    /// Parse a LiDAR point from byte array at given offset
    fn from_bytes(data: &[u8], offset: usize) -> Option<Self> {
        if offset + 26 > data.len() {
            return None;
        }

        // Helper to extract bytes at position
        let get_f32 = |pos: usize| {
            f32::from_le_bytes(data[offset + pos..offset + pos + 4].try_into().unwrap())
        };

        let get_f64 = |pos: usize| {
            f64::from_le_bytes(data[offset + pos..offset + pos + 8].try_into().unwrap())
        };

        Some(LidarPoint {
            x: get_f32(0),           // Bytes 0-3
            y: get_f32(4),           // Bytes 4-7
            z: get_f32(8),           // Bytes 8-11
            intensity: get_f32(12),  // Bytes 12-15
            tag: data[offset + 16],  // Byte 16
            line: data[offset + 17], // Byte 17
            timestamp: get_f64(18),  // Bytes 18-25
        })
    }
}

struct DriveWith3D {
    scan_subscription: Subscription<PointCloud2>,
    drive_publisher: Publisher<AckermannDriveStamped>,
}

impl DriveWith3D {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create node
        let node = executor.create_node("3d_lidar_drive")?;

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let publisher_clone = drive_publisher.clone();

        // Create original LiDAR subscriber with callback
        let scan_subscription =
            node.create_subscription::<PointCloud2, _>("/livox/lidar", move |msg: PointCloud2| {
                if let Err(e) = Self::process_and_publish_filtered(msg, &publisher_clone) {
                    eprintln!("Error during filtering process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
        })
    }
}

fn main() -> Result<(), RclrsError> {
    println!("Starting 3D LiDAR Drive");

    // Use the new executor pattern
    let mut executor = Context::default_from_env()?.create_basic_executor();

    // Create the filtering node instance
    let _3d_lidar_drive = DriveWith3D::new(&executor)?;

    // Spin the executor
    executor.spin(SpinOptions::default()).first_error()
}
