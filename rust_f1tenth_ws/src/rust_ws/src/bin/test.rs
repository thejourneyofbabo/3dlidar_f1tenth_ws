use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};
use visualization_msgs::msg::Marker;

// Define constants at the crate level or within the struct if logically tied
const MAX_RANGE: f32 = 5.0;
const MIN_RANGE: f32 = 0.3;
const VEHICLE_WIDTH: f32 = 0.4;
const ROI_ANGLE_DEG: f32 = 69.0; // degree
const MIN_GAP_RANGE: f32 = 1.5; // Minimum range to consider a point part of a gap
const EMA_ALPHA: f32 = 0.7; // Alpha for Exponential Moving Average filter
const LIDAR_TO_REAR: f32 = 0.27;
const WHEEL_BASE: f32 = 0.32;
const MIN_SPEED: f32 = 0.5;
const MAX_SPEED: f32 = 4.5;
const MAX_STEERING_RAD: f32 = PI / 4.0;

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    marker_publisher: Publisher<Marker>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let marker_publisher = node.create_publisher::<Marker>("/visualization_marker")?;
        let publisher_clone = drive_publisher.clone();
        let marker_clone = marker_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) =
                    Self::lidar_callback(msg, &publisher_clone, &marker_clone, &prev_best_clone)
                {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            marker_publisher,
            previous_best_point,
        })
    }

    fn process_lidar(msg: &LaserScan) -> Vec<f32> {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        // 3.Dynamic range adjustment for narrow environments
        let mut ranges = msg.ranges.clone();
        let mut min_dist = MAX_RANGE;
        let mut min_idx = 0;
        let mut max_detected_range = 0.0;

        // Find maximum detected range
        for range in ranges.iter() {
            if !range.is_nan() && !range.is_infinite() && *range > 0.0 && *range <= MAX_RANGE {
                if *range > max_detected_range {
                    max_detected_range = *range;
                }
            }
        }

        // Dynamic range adjustment for narrow environments
        let effective_max_range = if max_detected_range < MAX_RANGE {
            (max_detected_range * 0.8).max(MIN_RANGE * 2.0) // Ensure minimum usable range
        } else {
            MAX_RANGE
        };

        // Apply effective range and find minimum distance
        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > effective_max_range || range.is_nan() || range.is_infinite() {
                *range = effective_max_range;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        // Safety bubble logic
        if min_dist < MIN_RANGE {
            let bubble_radius = ((VEHICLE_WIDTH / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;
            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, ranges.len() - 1);

            for i in start_idx..=end_idx {
                ranges[i] = 0.0
            }
        }

        ranges
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32]) -> (usize, usize) {
        // Set Region of Interest(ROI)
        let roi_angle_rad = ROI_ANGLE_DEG * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize;
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        // Return the start index & end index of the max gap in free_space_ranges
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > MIN_GAP_RANGE;

            // Gap begin
            if is_free && !in_gap {
                gap_start = i;
                in_gap = true;
            } else if !is_free && in_gap {
                // End of current gap
                let gap_size = i - gap_start;
                if gap_size > max_gap_size {
                    max_gap_size = gap_size;
                    max_gap_start = gap_start;
                    max_gap_end = i - 1;
                }
                in_gap = false;
            }
        }

        if in_gap {
            let gap_size = roi_idx_end - gap_start + 1;
            if gap_size > max_gap_size {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }

        if max_gap_size == 0 {
            println!("Warning!: No gap found!!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        (max_gap_start, max_gap_end)
    }

    fn find_best_point(
        ranges: &[f32],
        gap_start: usize,
        gap_end: usize,
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> usize {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
        let mut weighted_sum = 0.0;
        let mut weight_total = 0.0;

        for i in gap_start..=gap_end {
            let weight = ranges[i].powi(2);
            weighted_sum += i as f32 * weight;
            weight_total += weight;
        }

        let best_point = if weight_total > 0.0 {
            (weighted_sum / weight_total) as usize
        } else {
            (gap_start + gap_end) / 2
        };

        // EMA Filter
        let ema_best = {
            let mut prev_lock = previous_best.lock().unwrap();
            let ema_result = match *prev_lock {
                Some(prev) => (EMA_ALPHA * (best_point as f32) + (1.0 - EMA_ALPHA) * (prev as f32))
                    .round() as usize,
                None => best_point,
            };

            *prev_lock = Some(ema_result);
            ema_result
        };

        ema_best
    }

    fn pure_pursuit(steer_ang_rad: &f32, lookahead_dist: f32) -> f32 {
        let bestpoint_x = lookahead_dist * f32::cos(*steer_ang_rad);
        let bestpoint_y = lookahead_dist * f32::sin(*steer_ang_rad);

        let lookahead_angle = f32::atan2(bestpoint_y, bestpoint_x + LIDAR_TO_REAR);
        let lookahead_rear = f32::sqrt((bestpoint_x + LIDAR_TO_REAR).powi(2) + bestpoint_y.powi(2));

        // Final Pure Pursuit Angle
        f32::atan2(2.0 * WHEEL_BASE * lookahead_angle.sin(), lookahead_rear)
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> (f32, f32) {
        // Calculate Steering-Angle & Speed

        // Convert best point index to steering angle in radians
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        // Get lookahead distance (capped at 3m)
        let best_lookahead = msg.ranges[best_point].min(3.0);

        let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);

        // Calculate initial steering ratio for adaptive lookahead
        let steering_ratio = (steer_ang_rad.abs() / MAX_STEERING_RAD).clamp(0.0, 1.0);

        // Reduce lookahead distance when steering more (for tighter turns)
        let adaptive_lookahead = lerp(best_lookahead, best_lookahead * 0.05, steering_ratio.sqrt());

        // Apply pure pursuit algorithm to get final steering angle
        let pure_pursuit_steer = Self::pure_pursuit(&steer_ang_rad, adaptive_lookahead);

        // Recalculate steering ratio based on final steering angle for speed control
        let steering_ratio = (pure_pursuit_steer.abs() / MAX_STEERING_RAD).clamp(0.0, 1.0);

        // Slow down when steering more (adaptive speed control)
        let drive_speed = lerp(MAX_SPEED, MIN_SPEED, steering_ratio);

        println!("=== DEBUG ===");
        println!("adaptive_lookahead: {}", adaptive_lookahead);
        println!("steering_ratio: {:.6}", steering_ratio);
        println!("final_speed: {}", drive_speed);
        println!("=============");

        (pure_pursuit_steer, drive_speed)
    }

    fn create_steering_arrow(steering_angle: f32, frame_id: &str) -> Marker {
        let mut marker = Marker::default();

        // Basic marker properties
        marker.header.frame_id = frame_id.to_string();
        // marker.header.stamp = // Leave default (will be filled by ROS)
        marker.id = 0;
        marker.type_ = 0; // ARROW type
        marker.action = 0; // ADD action

        // Arrow position (at vehicle center)
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.5; // Slightly above ground

        // Arrow orientation based on steering angle
        let yaw = steering_angle as f64;
        marker.pose.orientation.z = (yaw / 2.0).sin();
        marker.pose.orientation.w = (yaw / 2.0).cos();

        // Arrow size (length shows steering intensity)
        let arrow_length = (steering_angle.abs() / MAX_STEERING_RAD * 2.0 + 0.5).min(3.0) as f64;
        marker.scale.x = arrow_length; // Arrow length
        marker.scale.y = 0.1; // Arrow width
        marker.scale.z = 0.1; // Arrow height

        // Color: Red for left turn, Blue for right turn
        if steering_angle > 0.0 {
            // Left turn - Red
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else {
            // Right turn - Blue
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        marker.color.a = 0.8; // Transparency

        marker
    }

    fn create_speed_arrow(speed: f32, frame_id: &str) -> Marker {
        let mut marker = Marker::default();

        // Basic marker properties
        marker.header.frame_id = frame_id.to_string();
        // marker.header.stamp = // Leave default (will be filled by ROS)
        marker.id = 1; // Different ID from steering arrow
        marker.type_ = 0; // ARROW type
        marker.action = 0; // ADD action

        // Arrow position (slightly offset from steering arrow)
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.8; // Higher than steering arrow

        // Forward direction (no rotation)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Arrow size based on speed
        let speed_ratio = speed / MAX_SPEED;
        marker.scale.x = (speed_ratio * 3.0 + 0.5) as f64; // Arrow length
        marker.scale.y = 0.15; // Arrow width
        marker.scale.z = 0.15; // Arrow height

        // Color: Green gradient based on speed (darker = faster)
        marker.color.r = 0.0;
        marker.color.g = (speed_ratio * 0.8 + 0.2) as f32; // Green intensity
        marker.color.b = 0.0;
        marker.color.a = 0.8;

        marker
    }

    fn publish_visualization(
        marker_publisher: &Publisher<Marker>,
        steering_angle: f32,
        speed: f32,
        frame_id: &str,
    ) -> Result<(), Error> {
        // Publish steering arrow
        let steering_marker = Self::create_steering_arrow(steering_angle, frame_id);
        marker_publisher.publish(&steering_marker)?;

        // Publish speed arrow
        let speed_marker = Self::create_speed_arrow(speed, frame_id);
        marker_publisher.publish(&speed_marker)?;

        Ok(())
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        marker_publisher: &Publisher<Marker>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
    ) -> Result<(), Error> {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        let processed_ranges = Self::process_lidar(&msg);

        let (max_gap_start, max_gap_end) = Self::find_max_gap(&msg, &processed_ranges);

        let best_point_idx = Self::find_best_point(
            &processed_ranges,
            max_gap_start,
            max_gap_end,
            previous_best_point,
        );

        let (steering_angle, drive_speed) = Self::vehicle_control(&msg, best_point_idx);

        // Publish drive command
        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;
        drive_publisher.publish(&drive_msg)?;

        // Publish visualization arrows
        Self::publish_visualization(
            marker_publisher,
            steering_angle,
            drive_speed,
            &msg.header.frame_id,
        )?;

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust (with RViz visualization)");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
