use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use nalgebra::Point3;
use rand::prelude::*;
use rand::rng;
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

    /// Convert to nalgebra Point3 for mathematical operations
    fn to_point3(&self) -> Point3<f32> {
        Point3::new(self.x, self.y, self.z)
    }
}

/// Represents a 3D plane using the equation: ax + by + cz + d = 0
#[derive(Debug, Clone)]
struct Plane {
    a: f32,
    b: f32,
    c: f32,
    d: f32,
}

impl Plane {
    /// Create a plane from three non-collinear points
    fn from_three_points(p1: &Point3<f32>, p2: &Point3<f32>, p3: &Point3<f32>) -> Option<Self> {
        let v1 = p2 - p1;
        let v2 = p3 - p1;

        // Cross product to get normal vector
        let normal = v1.cross(&v2);

        // Check if points are collinear (cross product is zero)
        if normal.norm() < 1e-6 {
            return None;
        }

        let normal = normal.normalize();

        // Calculate d using point p1
        let d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);

        Some(Plane {
            a: normal.x,
            b: normal.y,
            c: normal.z,
            d,
        })
    }

    /// Calculate distance from a point to this plane (optimized)
    fn distance_to_point(&self, point: &Point3<f32>) -> f32 {
        let numerator = (self.a * point.x + self.b * point.y + self.c * point.z + self.d).abs();
        let denominator_sq = self.a * self.a + self.b * self.b + self.c * self.c;
        numerator / denominator_sq.sqrt()
    }
}

/// Optimized RANSAC ground plane extraction
struct RansacGroundExtractor {
    max_iterations: usize,
    distance_threshold: f32,
    min_inliers: usize,
    early_stop_threshold: f32, // Early stopping for performance
}

impl RansacGroundExtractor {
    fn new(max_iterations: usize, distance_threshold: f32, min_inliers: usize) -> Self {
        Self {
            max_iterations,
            distance_threshold,
            min_inliers,
            early_stop_threshold: 0.7, // Stop if 70% of points are inliers
        }
    }

    /// Extract ground plane using optimized RANSAC algorithm
    fn extract_ground(&self, points: &[LidarPoint]) -> (Vec<LidarPoint>, Vec<LidarPoint>) {
        if points.len() < 3 {
            return (Vec::new(), points.to_vec());
        }

        // Pre-filter points that are likely to be ground based on Z coordinate
        let ground_candidates: Vec<&LidarPoint> = points
            .iter()
            .filter(|p| p.z >= -1.0 && p.z <= 0.5) // Rough ground height filter
            .collect();

        if ground_candidates.len() < 3 {
            return (Vec::new(), points.to_vec());
        }

        let mut rng = rng();
        let mut best_plane: Option<Plane> = None;
        let mut best_inliers = Vec::new();
        let early_stop_count = (points.len() as f32 * self.early_stop_threshold) as usize;

        for iteration in 0..self.max_iterations {
            // Randomly sample 3 points from ground candidates for faster convergence
            let candidate_indices: Vec<usize> = (0..ground_candidates.len())
                .choose_multiple(&mut rng, 3)
                .into_iter()
                .collect();

            let p1 = ground_candidates[candidate_indices[0]].to_point3();
            let p2 = ground_candidates[candidate_indices[1]].to_point3();
            let p3 = ground_candidates[candidate_indices[2]].to_point3();

            // Try to fit a plane
            if let Some(plane) = Plane::from_three_points(&p1, &p2, &p3) {
                // Find inliers for this plane
                let mut inliers = Vec::new();

                for (i, point) in points.iter().enumerate() {
                    let distance = plane.distance_to_point(&point.to_point3());
                    if distance <= self.distance_threshold {
                        inliers.push(i);
                    }
                }

                // Update best model if this one is better
                if inliers.len() > best_inliers.len() && inliers.len() >= self.min_inliers {
                    best_plane = Some(plane);
                    best_inliers = inliers;

                    // Early stopping for performance
                    if best_inliers.len() >= early_stop_count {
                        println!(
                            "RANSAC early stop at iteration {} with {} inliers",
                            iteration + 1,
                            best_inliers.len()
                        );
                        break;
                    }
                }
            }
        }

        // Separate ground and non-ground points
        let mut ground_points = Vec::new();
        let mut non_ground_points = Vec::new();

        for (i, point) in points.iter().enumerate() {
            if best_inliers.contains(&i) {
                ground_points.push(point.clone());
            } else {
                non_ground_points.push(point.clone());
            }
        }

        println!("RANSAC Results:");
        println!("  Total points: {}", points.len());
        println!(
            "  Ground points: {} ({:.1}%)",
            ground_points.len(),
            100.0 * ground_points.len() as f32 / points.len() as f32
        );
        println!("  Non-ground points: {}", non_ground_points.len());

        if let Some(plane) = best_plane {
            println!(
                "  Best plane: {:.3}x + {:.3}y + {:.3}z + {:.3} = 0",
                plane.a, plane.b, plane.c, plane.d
            );
        }

        (ground_points, non_ground_points)
    }
}

// Fast ground extraction using simple height-based filtering (fallback option)
struct FastGroundExtractor {
    ground_height: f32,
    tolerance: f32,
}

impl FastGroundExtractor {
    fn new(ground_height: f32, tolerance: f32) -> Self {
        Self {
            ground_height,
            tolerance,
        }
    }

    fn extract_ground(&self, points: &[LidarPoint]) -> (Vec<LidarPoint>, Vec<LidarPoint>) {
        let mut ground_points = Vec::new();
        let mut non_ground_points = Vec::new();

        for point in points {
            if (point.z - self.ground_height).abs() <= self.tolerance {
                ground_points.push(point.clone());
            } else {
                non_ground_points.push(point.clone());
            }
        }

        println!("Fast Ground Extraction Results:");
        println!("  Total points: {}", points.len());
        println!(
            "  Ground points: {} ({:.1}%)",
            ground_points.len(),
            100.0 * ground_points.len() as f32 / points.len() as f32
        );
        println!("  Non-ground points: {}", non_ground_points.len());

        (ground_points, non_ground_points)
    }
}

#[allow(dead_code)]
struct DriveWith3D {
    scan_subscription: Subscription<PointCloud2>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    ground_extractor: RansacGroundExtractor,
    fast_extractor: FastGroundExtractor,
    use_ransac: bool,
}

impl DriveWith3D {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        // Create node
        let node = executor.create_node("lidar_3d_drive")?;

        // Create publishers
        let filtered_publisher = node.create_publisher::<PointCloud2>("/livox/lidar_filted")?;
        let ground_publisher = node.create_publisher::<PointCloud2>("/livox/lidar_ground")?;
        let non_ground_publisher =
            node.create_publisher::<PointCloud2>("/livox/lidar_obstacles")?;
        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        // Clone publishers for callback use
        let filter_pub_clone = filtered_publisher.clone();
        let ground_pub_clone = ground_publisher.clone();
        let non_ground_pub_clone = non_ground_publisher.clone();
        let drive_pub_clone = drive_publisher.clone();

        // Initialize ground extractors
        let ground_extractor = RansacGroundExtractor::new(
            50,   // Reduced iterations for speed
            0.15, // distance_threshold
            30,   // Reduced min_inliers
        );

        let fast_extractor = FastGroundExtractor::new(-0.2, 0.3); // ground at -0.2m ± 0.3m

        // Create original LiDAR subscriber with callback
        let scan_subscription =
            node.create_subscription::<PointCloud2, _>("/livox/lidar", move |msg: PointCloud2| {
                if let Err(e) = Self::lidar_callback(
                    msg,
                    &filter_pub_clone,
                    &ground_pub_clone,
                    &non_ground_pub_clone,
                    &drive_pub_clone,
                ) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            ground_extractor,
            fast_extractor,
            use_ransac: false, // Start with fast method, switch to true for RANSAC
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
        points: &[LidarPoint],
        original_msg: &PointCloud2,
        _frame_suffix: &str,
    ) -> PointCloud2 {
        // Generate byte data for all points
        let mut data = Vec::with_capacity(points.len() * original_msg.point_step as usize);
        for point in points.iter() {
            data.extend_from_slice(&point.to_bytes());
        }

        // Create new header with filtered frame_id
        let filtered_header = original_msg.header.clone();

        PointCloud2 {
            header: filtered_header,
            height: original_msg.height,
            width: points.len() as u32,
            fields: original_msg.fields.clone(),
            is_bigendian: original_msg.is_bigendian,
            point_step: original_msg.point_step,
            row_step: (points.len() as u32) * original_msg.point_step,
            data,
            is_dense: true,
        }
    }

    fn lidar_callback(
        msg: PointCloud2,
        point_publisher: &Publisher<PointCloud2>,
        ground_publisher: &Publisher<PointCloud2>,
        non_ground_publisher: &Publisher<PointCloud2>,
        drive_publisher: &Publisher<AckermannDriveStamped>,
    ) -> Result<(), Error> {
        let start_time = std::time::Instant::now();

        // 1. Parse original 3D points
        let lidar_points = Self::parse_pointcloud2(&msg);
        let original_count = lidar_points.len();

        // 2. Apply aggressive ROI filter for performance
        let roi_filtered_points: Vec<LidarPoint> = lidar_points
            .into_iter()
            .filter(|point| {
                point.z >= -1.5 && point.z <= 0.0  // Tighter Z range
                    && point.x >= -5.0 && point.x <= 8.0   // Smaller X range
                    && point.y >= -4.0 && point.y <= 4.0   // Smaller Y range
                    && point.x.abs() + point.y.abs() < 10.0 // Distance filter
            })
            .collect();

        println!(
            "Original: {}, ROI filtered: {} ({:.1}%)",
            original_count,
            roi_filtered_points.len(),
            100.0 * roi_filtered_points.len() as f32 / original_count as f32
        );

        // 3. Choose extraction method based on point count
        let use_ransac = roi_filtered_points.len() < 1000; // Use RANSAC only for small point clouds

        let (ground_points, non_ground_points) = if use_ransac {
            println!("Using RANSAC extraction");
            let extractor = RansacGroundExtractor::new(30, 0.15, 20); // Very fast settings
            extractor.extract_ground(&roi_filtered_points)
        } else {
            println!("Using fast height-based extraction");
            let extractor = FastGroundExtractor::new(-0.2, 0.25);
            extractor.extract_ground(&roi_filtered_points)
        };

        // 4. Filter obstacles more aggressively
        let obstacle_points: Vec<LidarPoint> = non_ground_points
            .into_iter()
            .filter(|point| {
                point.z >= -0.3 && point.z <= 1.5  // Above ground, below ceiling
                    && point.x >= 0.5 && point.x <= 4.0  // In front only
                    && point.y.abs() <= 1.5 // Narrow corridor
            })
            .collect();

        let processing_time = start_time.elapsed();
        println!(
            "Processing time: {:.1}ms, Obstacles: {}",
            processing_time.as_millis(),
            obstacle_points.len()
        );

        // 5. Publish results (only if needed for debugging)
        if ground_points.len() > 0 {
            let ground_msg = Self::create_filtered_pointcloud2(&ground_points, &msg, "ground");
            ground_publisher.publish(ground_msg)?;
        }

        if obstacle_points.len() > 0 {
            let obstacle_msg =
                Self::create_filtered_pointcloud2(&obstacle_points, &msg, "obstacles");
            non_ground_publisher.publish(obstacle_msg)?;
        }

        // 6. Drive command calculation
        let (steering_angle, drive_speed) = Self::calculate_drive_command(&obstacle_points);

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        Ok(())
    }

    /// Simple obstacle avoidance based on obstacle points
    fn calculate_drive_command(obstacle_points: &[LidarPoint]) -> (f32, f32) {
        if obstacle_points.is_empty() {
            return (0.0, 1.5); // No obstacles, go faster
        }

        // Count obstacles in left, center, and right regions
        let mut left_count = 0;
        let mut center_count = 0;
        let mut right_count = 0;

        for point in obstacle_points {
            if point.y > 0.3 {
                left_count += 1;
            } else if point.y < -0.3 {
                right_count += 1;
            } else {
                center_count += 1;
            }
        }

        // Simple decision logic
        let steering_angle = if center_count > 3 {
            if left_count < right_count {
                0.4 // Turn left
            } else {
                -0.4 // Turn right
            }
        } else {
            0.0 // Go straight
        };

        let drive_speed = if center_count > 5 {
            0.3 // Slow down significantly
        } else if obstacle_points.len() > 10 {
            0.7 // Moderate speed
        } else {
            1.2 // Normal speed
        };

        (steering_angle, drive_speed)
    }
}

fn main() -> Result<(), RclrsError> {
    println!("Starting Optimized 3D LiDAR Drive");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _3d_lidar_drive = DriveWith3D::new(&executor)?;

    executor.spin(SpinOptions::default()).first_error()
}
