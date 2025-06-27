use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

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
const MAX_SPEED: f32 = 3.5;
const MAX_STEERING_RAD: f32 = PI / 4.0;

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let publisher_clone = drive_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(msg, &publisher_clone, &prev_best_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
        })
    }

    fn process_lidar(msg: &LaserScan) -> Vec<f32> {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        let mut ranges = msg.ranges.clone();
        let mut min_dist = MAX_RANGE;
        let mut min_idx = 0;

        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > MAX_RANGE || range.is_nan() || range.is_infinite() {
                *range = MAX_RANGE;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

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
            let weight = ranges[i].powi(1);
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
        //println!("Best Point = {}", ema_best);

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
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let best_lookahead = msg.ranges[best_point].min(3.0);

        let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);

        let steering_ratio = (steer_ang_rad.abs() / MAX_STEERING_RAD).clamp(0.0, 1.0);

        let adaptive_lookahead = lerp(best_lookahead, best_lookahead * 0.05, steering_ratio.sqrt());

        let pure_pursuit_steer = Self::pure_pursuit(&steer_ang_rad, adaptive_lookahead);

        let steering_ratio = (pure_pursuit_steer.abs() / MAX_STEERING_RAD).clamp(0.0, 1.0);

        // Adaptive speed control based on steering
        let drive_speed = lerp(MAX_SPEED, MIN_SPEED, steering_ratio);

        println!("=== DEBUG ===");
        println!("adaptive_lookahead: {}", adaptive_lookahead);
        println!("steering_ratio: {:.6}", steering_ratio.sqrt());
        println!("final_speed: {}", drive_speed);
        println!("=============");

        (pure_pursuit_steer, drive_speed)
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
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

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        // Publish Drive message
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
