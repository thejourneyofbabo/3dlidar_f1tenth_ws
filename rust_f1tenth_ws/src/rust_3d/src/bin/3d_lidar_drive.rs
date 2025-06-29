use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use nalgebra;
use rclrs::*;
use sensor_msgs::msg::PointCloud2;

/// Represents a 3D LiDAR point with all sensor data
#[allow(dead_code)]
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

#[allow(dead_code)]
struct DriveWith3D {
    scan_subscription: Subscription<PointCloud2>,
    drive_publisher: Publisher<AckermannDriveStamped>,
}

impl DriveWith3D {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create node
        let node = executor.create_node("lidar_3d_drive")?;

        // Create publishers
        let filtered_publisher = node.create_publisher::<PointCloud2>("/livox/lidar_filted")?;
        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        // Clone publisher for callback use
        let filter_pub_clone = filtered_publisher.clone();
        let drive_pub_clone = drive_publisher.clone();

        // Create original LiDAR subscriber with callback
        let scan_subscription =
            node.create_subscription::<PointCloud2, _>("/livox/lidar", move |msg: PointCloud2| {
                if let Err(e) = Self::lidar_callback(msg, &filter_pub_clone, &drive_pub_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
        })
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
    fn create_filtered_pointcloud2(
        points: Vec<LidarPoint>,
        original_msg: &PointCloud2,
        _frame_suffix: &str,
    ) -> PointCloud2 {
        // Generate byte data for all points
        let mut data = Vec::with_capacity(points.len() * original_msg.point_step as usize);
        for point in points.iter() {
            data.extend_from_slice(&point.to_bytes());
        }

        // Create new header with filtered frame_id
        let mut filtered_header = original_msg.header.clone();
        //filtered_header.frame_id = format!("{}_{}", original_msg.header.frame_id, frame_suffix);
        //filtered_header.frame_id = format!("{}_filted", original_msg.header.frame_id);
        //filtered_header.frame_id = "filted_frame".to_string();

        PointCloud2 {
            header: filtered_header,
            height: original_msg.height,         // Keep original height
            width: points.len() as u32,          // Update width to filtered count
            fields: original_msg.fields.clone(), // Copy fields from original!
            is_bigendian: original_msg.is_bigendian, // Keep same endianness
            point_step: original_msg.point_step, // Same bytes per point
            row_step: (points.len() as u32) * original_msg.point_step,
            data,
            is_dense: true, // We filtered out invalid points
        }
    }

    fn lidar_callback(
        msg: PointCloud2,
        point_publisher: &Publisher<PointCloud2>,
        drive_publisher: &Publisher<AckermannDriveStamped>,
    ) -> Result<(), Error> {
        // 1. Parse original 3D points
        let lidar_points = Self::parse_pointcloud2(&msg);
        let original_count = lidar_points.len();

        // 2. PointCloud ROI Set
        let filtered_points: Vec<LidarPoint> = lidar_points
            .into_iter()
            .filter(|point| {
                point.z >= -0.1
                    && point.z <= 0.0
                    && point.x >= -5.0
                    && point.x <= 8.0
                    && point.y >= -4.0
                    && point.y <= 4.0
            }) // PointCloud ROI setup
            .collect();

        println!("Original point count: {}", original_count);
        println!("Filtered point count: {}", filtered_points.len());

        // 3. Create new PointCloud2 message
        let filtered_msg = Self::create_filtered_pointcloud2(filtered_points, &msg, "filted");

        // 4. Publish to filtered topic
        point_publisher.publish(filtered_msg)?;

        let (steering_angle, drive_speed) = (0.0, 1.0);

        let mut drive_msg = AckermannDriveStamped::default();

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        Ok(())
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
