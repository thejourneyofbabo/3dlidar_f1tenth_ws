// Working code

use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
    scan_count: Arc<Mutex<u32>>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        println!("Node created successfully");

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let publisher_clone = drive_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();

        let scan_count = Arc::new(Mutex::new(0));
        let scan_count_clone = scan_count.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                let mut count = scan_count_clone.lock().unwrap();
                *count += 1;

                if *count % 10 == 1 {
                    // Print every 10th scan to avoid spam
                    println!(
                        "=== Scan #{} received with {} ranges ===",
                        *count,
                        msg.ranges.len()
                    );
                }

                if let Err(e) =
                    Self::lidar_callback(msg, &publisher_clone, &prev_best_clone, *count)
                {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        println!("Subscription to /scan created");
        println!("Publisher to /drive created");

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
            scan_count,
        })
    }

    fn process_lidar(msg: &LaserScan) -> Vec<f32> {
        let max_range: f32 = 3.0;
        let min_range: f32 = 0.3;
        let vehicle_width: f32 = 0.4;

        let mut ranges = msg.ranges.clone();
        let mut min_dist = max_range;
        let mut min_idx = 0;

        // Find minimum distance
        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > max_range || range.is_nan() || range.is_infinite() {
                *range = max_range;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        // Apply safety bubble
        if min_dist < min_range {
            let bubble_radius = ((vehicle_width / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, ranges.len() - 1);

            println!(
                "  Safety bubble: min_dist={:.2}m at idx={}, bubble range: {}-{}",
                min_dist, min_idx, start_idx, end_idx
            );

            for i in start_idx..=end_idx {
                ranges[i] = 0.0;
            }
        }

        ranges
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32]) -> (usize, usize) {
        // Set Region of Interest(ROI)
        let roi_angle_deg = 67.0;
        let roi_angle_rad = roi_angle_deg * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize;

        // Safer calculation of center
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        println!(
            "  ROI: center={}, range={}..{} (total {} indices)",
            mid_lidar_idx,
            roi_idx_start,
            roi_idx_end,
            ranges.len()
        );

        // Find the largest gap
        let min_range = 0.5;
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > min_range;

            if is_free && !in_gap {
                // Start of a new gap
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

        // Check if we ended while still in a gap
        if in_gap {
            let gap_size = roi_idx_end - gap_start + 1;
            if gap_size > max_gap_size {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }

        println!(
            "  Max gap: start={}, end={}, size={}",
            max_gap_start, max_gap_end, max_gap_size
        );

        // If no gap found, return middle of ROI
        if max_gap_size == 0 {
            println!("  WARNING: No gap found! Using center of ROI");
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
        // Find the furthest point in the gap
        let mut best_point = (gap_start + gap_end) / 2;
        let mut max_dist = 0.0;
        let alpha = 0.3;

        for i in gap_start..=gap_end {
            if ranges[i] > max_dist {
                max_dist = ranges[i];
                best_point = i;
            }
        }

        println!(
            "  Best point in gap: idx={}, dist={:.2}m",
            best_point, max_dist
        );

        // Apply exponential moving average filter
        let ema_best = {
            let mut prev_lock = previous_best.lock().unwrap();
            let ema_result = if let Some(prev) = *prev_lock {
                (alpha * best_point as f32 + (1.0 - alpha) * prev as f32) as usize
            } else {
                best_point
            };
            *prev_lock = Some(ema_result);
            ema_result
        };

        println!("  EMA filtered best point: {}", ema_best);

        ema_best
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> (f32, f32) {
        // Calculate steering angle in radians
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        // Clamp steering angle to reasonable limits
        let steer_ang_rad = steer_ang_rad.clamp(-0.4, 0.4);

        // Convert to degrees just for the speed logic
        let steer_ang_deg = steer_ang_rad.abs() * 180.0 / PI;

        let drive_speed = match steer_ang_deg {
            n if n < 5.0 => 2.0,
            n if n < 10.0 => 1.5,
            n if n < 15.0 => 1.2,
            _ => 0.8,
        };

        println!(
            "  Control: steer={:.3} rad ({:.1}°), speed={:.1} m/s",
            steer_ang_rad,
            steer_ang_rad * 180.0 / PI,
            drive_speed
        );

        (steer_ang_rad, drive_speed)
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        scan_num: u32,
    ) -> Result<(), Error> {
        // Only print detailed info every 10th scan
        let verbose = scan_num % 10 == 1;

        if verbose {
            println!("Processing scan #{}", scan_num);
            println!(
                "  Angle range: {:.2} to {:.2} rad",
                msg.angle_min, msg.angle_max
            );
            println!("  Angle increment: {:.4} rad", msg.angle_increment);
        }

        // Process the lidar data
        let processed_ranges = Self::process_lidar(&msg);

        // Find the maximum gap
        let (max_gap_start, max_gap_end) = Self::find_max_gap(&msg, &processed_ranges);

        // Find the best point in the gap
        let best_point_idx = Self::find_best_point(
            &processed_ranges,
            max_gap_start,
            max_gap_end,
            previous_best_point,
        );

        // Calculate control commands
        let (steering_angle, drive_speed) = Self::vehicle_control(&msg, best_point_idx);

        // Create and publish the drive message
        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        if verbose {
            println!(
                "  Publishing: speed={:.2}, steer={:.3} rad",
                drive_msg.drive.speed, drive_msg.drive.steering_angle
            );
        }

        drive_publisher.publish(&drive_msg)?;

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("=== F1Tenth Gap Follow Node with Rust ===");
    println!("Starting up...");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;

    println!("Node initialized, starting executor spin...");
    println!("Waiting for /scan messages...");

    executor.spin(SpinOptions::default()).first_error()
}
