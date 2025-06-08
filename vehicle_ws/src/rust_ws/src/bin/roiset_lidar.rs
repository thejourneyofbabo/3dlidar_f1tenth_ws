use anyhow::{Error, Result};
use rclrs::{self, Context, Publisher};
use sensor_msgs::msg::PointCloud2;
use std::env;
use std::sync::Arc;

/// Represents a 3D LiDAR point with all sensor data
#[derive(Debug, Clone)] // Added Clone for easier use
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

        // Extract X coordinate (4 bytes, little-endian)
        let x = f32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);

        // Extract Y coordinate (4 bytes, little-endian)
        let y = f32::from_le_bytes([
            data[offset + 4],
            data[offset + 5],
            data[offset + 6],
            data[offset + 7],
        ]);

        // Extract Z coordinate (4 bytes, little-endian)
        let z = f32::from_le_bytes([
            data[offset + 8],
            data[offset + 9],
            data[offset + 10],
            data[offset + 11],
        ]);

        // Extract intensity (4 bytes, little-endian)
        let intensity = f32::from_le_bytes([
            data[offset + 12],
            data[offset + 13],
            data[offset + 14],
            data[offset + 15],
        ]);

        // Extract tag (1 byte)
        let tag = data[offset + 16];

        // Extract line (1 byte)
        let line = data[offset + 17];

        // Extract timestamp (8 bytes, little-endian)
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

    /// Convert point to byte array for PointCloud2 message
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(26);

        // X, Y, Z coordinates (4 bytes each)
        bytes.extend_from_slice(&self.x.to_le_bytes());
        bytes.extend_from_slice(&self.y.to_le_bytes());
        bytes.extend_from_slice(&self.z.to_le_bytes());

        // Intensity (4 bytes)
        bytes.extend_from_slice(&self.intensity.to_le_bytes());

        // Tag and line (1 byte each)
        bytes.push(self.tag);
        bytes.push(self.line);

        // Timestamp (8 bytes)
        bytes.extend_from_slice(&self.timestamp.to_le_bytes());

        bytes
    }
}

/// Parse PointCloud2 message into vector of LiDAR points
fn parse_pointcloud2(msg: &PointCloud2) -> Vec<LidarPoint> {
    let mut points = Vec::with_capacity(msg.data.len() / msg.point_step as usize);
    let point_step = msg.point_step as usize;

    // Process each point in the point cloud
    for i in (0..msg.data.len()).step_by(point_step) {
        if let Some(point) = LidarPoint::from_bytes(&msg.data, i) {
            points.push(point);
        }
    }

    points
}

/// Create filtered PointCloud2 message from filtered points
fn create_filtered_pointcloud2(points: Vec<LidarPoint>, original_msg: &PointCloud2) -> PointCloud2 {
    // Generate byte data for all points
    let mut data = Vec::with_capacity(points.len() * original_msg.point_step as usize);
    for point in points.iter() {
        data.extend_from_slice(&point.to_bytes());
    }

    // Create new header with filtered frame_id
    let mut filtered_header = original_msg.header.clone();
    filtered_header.frame_id = format!("{}_filted", original_msg.header.frame_id);

    PointCloud2 {
        header: filtered_header,
        height: original_msg.height,             // Keep original height
        width: points.len() as u32,              // Update width to filtered count
        fields: original_msg.fields.clone(),     // Copy fields from original!
        is_bigendian: original_msg.is_bigendian, // Keep same endianness
        point_step: original_msg.point_step,     // Same bytes per point
        row_step: (points.len() as u32) * original_msg.point_step,
        data,
        is_dense: true, // We filtered out invalid points
    }
}

/// Process incoming point cloud and publish filtered version
fn process_and_publish_filtered(
    msg: PointCloud2,
    publisher: &Arc<Publisher<PointCloud2>>,
) -> Result<(), Error> {
    // 1. Parse original 3D points
    let lidar_points = parse_pointcloud2(&msg);
    let original_count = lidar_points.len();

    // 2. Apply Z-axis filtering - no need to convert structs!
    let filtered_points: Vec<LidarPoint> = lidar_points
        .into_iter()
        .filter(|point| point.z >= -0.1 && point.z <= 0.0) // Z-axis filtering
        .collect();

    println!("Original point count: {}", original_count);
    println!("Filtered point count: {}", filtered_points.len());

    // 3. Create new PointCloud2 message
    let filtered_msg = create_filtered_pointcloud2(filtered_points, &msg);

    // 4. Publish to filtered topic
    publisher.publish(filtered_msg)?;

    println!("Filtered point cloud published successfully!");

    Ok(())
}

fn main() -> Result<(), Error> {
    println!("LiDAR Filtered Publisher Node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_filtered_publisher")?;

    // Create filtered point cloud publisher
    let filtered_publisher =
        node.create_publisher::<PointCloud2>("/livox/lidar_filted", rclrs::QOS_PROFILE_DEFAULT)?;
    let filtered_publisher = Arc::new(filtered_publisher);

    // Create original LiDAR subscriber
    let publisher_clone = Arc::clone(&filtered_publisher);
    let _subscriber = node.create_subscription::<PointCloud2, _>(
        "/livox/lidar",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PointCloud2| {
            if let Err(e) = process_and_publish_filtered(msg, &publisher_clone) {
                eprintln!("Error during filtering process: {}", e);
            }
        },
    )?;

    println!("Subscribe topic: /livox/lidar");
    println!("Publish topic: /livox/lidar_filted");
    println!("Starting filtered conversion...");

    rclrs::spin(node).map_err(|err| err.into())
}
