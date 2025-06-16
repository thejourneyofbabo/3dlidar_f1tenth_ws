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
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        println!("[DEBUG] Creating ROS2 node: follow_the_gap_node");
        let node = executor.create_node("follow_the_gap_node")?;
        println!("[DEBUG] Node created successfully");

        println!("[DEBUG] Creating publisher for /drive topic");
        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let publisher_clone = drive_publisher.clone();
        println!("[DEBUG] Publisher created successfully");

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();
        println!("[DEBUG] Initialized previous_best_point tracking");

        println!("[DEBUG] Creating subscription to /scan topic");
        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                println!(
                    "[DEBUG] Received LaserScan message with {} ranges",
                    msg.ranges.len()
                );
                if let Err(e) = Self::lidar_callback(msg, &publisher_clone, &prev_best_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;
        println!("[DEBUG] Subscription created successfully");

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
        println!("[DEBUG] Processing LiDAR data...");
        let max_range: f32 = 5.0;
        let min_range: f32 = max_range;
        let vehicle_width: f32 = 0.4;

        let mut ranges = msg.ranges.clone();
        let mut min_dist = max_range;
        let mut min_idx = 0;

        //let ranges: Vec<f32> = msg
        //    .ranges
        //    .iter()
        //    .map(|&range| if range <= max_range { range } else { max_range })
        //    .collect();

        println!("[DEBUG] Finding closest obstacle...");
        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > max_range || range.is_nan() || range.is_infinite() {
                *range = max_range;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }
        println!(
            "[DEBUG] Closest obstacle: {:.2}m at index {}",
            min_dist, min_idx
        );

        if min_dist < min_range {
            println!("[DEBUG] Obstacle too close! Creating safety bubble");
            let bubble_radius = ((vehicle_width / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;
            println!("[DEBUG] Bubble size: {} indices", bubble_size);

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, ranges.len() - 1);
            println!("[DEBUG] Bubble range: indices {} to {}", start_idx, end_idx);

            for i in start_idx..=end_idx {
                ranges[i] = 0.0
            }
        }

        println!("[DEBUG] LiDAR processing complete");
        ranges
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32]) -> (usize, usize) {
        // Set Region of Interest(ROI)
        let roi_angle_deg = 67.0; // degree
        let roi_angle_rad = roi_angle_deg * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize; // ROI angle steps
                                                                              // for left & right
        let mid_lidar_idx = ranges.len() / 2;
        //let (roi_idx_start, roi_idx_end) = (
        //    mid_lidar_idx - roi_angle_steps,
        //    mid_lidar_idx + roi_angle_steps,
        //);
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        println!(
            "[DEBUG] ROI: angle={}°, indices {} to {}",
            roi_angle_deg, roi_idx_start, roi_idx_end
        );

        // Return the start index & end index of the max gap in free_space_ranges
        let min_range = 1.5;
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;
        let mut gap_count = 0;

        println!(
            "[DEBUG] Searching for gaps with min_range threshold: {}m",
            min_range
        );
        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > min_range;
            //let was_free = i > roi_idx_start && ranges[i - 1] > min_range;

            // Gap begin
            if is_free && !in_gap {
                gap_start = i;
                in_gap = true;
                gap_count += 1;
                println!("[DEBUG] Gap {} started at index {}", gap_count, i);
            } else if !is_free && in_gap {
                // End of current gap
                let gap_size = i - gap_start;
                println!(
                    "[DEBUG] Gap {} ended at index {}, size: {} indices",
                    gap_count,
                    i - 1,
                    gap_size
                );
                if gap_size > max_gap_size {
                    max_gap_size = gap_size;
                    max_gap_start = gap_start;
                    max_gap_end = i - 1;
                    println!("[DEBUG] New largest gap found!");
                }
                in_gap = false;
            }
        }

        if in_gap {
            let gap_size = roi_idx_end - gap_start + 1;
            println!(
                "[DEBUG] Gap {} extends to ROI end, size: {} indices",
                gap_count, gap_size
            );
            if gap_size > max_gap_size {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
                println!("[DEBUG] New largest gap found!");
            }
        }

        if max_gap_size == 0 {
            println!("Warngin!: No gap found!!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        } else {
            println!(
                "[DEBUG] Max gap: indices {} to {}, size: {}",
                max_gap_start, max_gap_end, max_gap_size
            );
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
        println!(
            "[DEBUG] Finding best point in gap [{}, {}]",
            gap_start, gap_end
        );
        let mut best_point = (gap_start + gap_end) / 2;
        let mut max_dist = 0.0;
        let alpha = 0.8;

        for i in gap_start..=gap_end {
            if ranges[i] > max_dist {
                max_dist = ranges[i];
                best_point = i;
            }
        }
        println!(
            "[DEBUG] Furthest point: index {}, distance: {:.2}m",
            best_point, max_dist
        );

        // EMA Filter
        let ema_best = {
            let mut prev_lock = previous_best.lock().unwrap();
            let ema_result = if let Some(prev) = *prev_lock {
                let filtered = (alpha * best_point as f32 + (1.0 - alpha) * prev as f32) as usize;
                println!(
                    "[DEBUG] EMA filter: prev={}, current={}, filtered={}",
                    prev, best_point, filtered
                );
                filtered
            } else {
                println!(
                    "[DEBUG] EMA filter: no previous value, using current={}",
                    best_point
                );
                best_point
            };
            *prev_lock = Some(ema_result);
            ema_result
        };

        println!("[DEBUG] Final best point: index {}", ema_best);
        ema_best
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> (f32, f32) {
        // Calculate Steering-Angle & Speed
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;
        println!(
            "[DEBUG] Raw steering angle: {:.3} rad ({:.1}°)",
            steer_ang_rad,
            steer_ang_rad * 180.0 / PI
        );

        let steer_ang_rad = steer_ang_rad.clamp(-0.4, 0.4);
        let steer_ang_deg = steer_ang_rad.abs() * 180.0 / PI;
        println!(
            "[DEBUG] Clamped steering angle: {:.3} rad ({:.1}°)",
            steer_ang_rad,
            steer_ang_rad * 180.0 / PI
        );

        let drive_speed = match steer_ang_deg {
            n if n < 5.0 => 2.0,
            n if n < 10.0 => 1.5,
            n if n < 15.0 => 1.2,
            _ => 0.8,
        };
        println!(
            "[DEBUG] Speed decision: steering_deg={:.1}° → speed={:.1}m/s",
            steer_ang_deg, drive_speed
        );

        (steer_ang_rad, drive_speed)
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
    ) -> Result<(), Error> {
        println!("\n[DEBUG] ========== NEW LIDAR CALLBACK ==========");
        println!(
            "[DEBUG] LaserScan: {} ranges, angle_min={:.3}, angle_max={:.3}, angle_increment={:.6}",
            msg.ranges.len(),
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment
        );

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        let processed_ranges = Self::process_lidar(&msg);

        // Todo
        // Find closest point to LiDAR
        let (max_gap_start, max_gap_end) = Self::find_max_gap(&msg, &processed_ranges);

        // Eliminate all points inside 'bubble' (set them to zero)
        // Find max length gap

        // Find the best point in the gap
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

        println!(
            "[DEBUG] Publishing drive command: steering={:.3}rad, speed={:.1}m/s",
            steering_angle, drive_speed
        );
        drive_publisher.publish(&drive_msg)?;

        // Publish Drive message
        println!("[DEBUG] ========== CALLBACK COMPLETE ==========\n");
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust");
    println!("[DEBUG] Initializing ROS2 context...");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    println!("[DEBUG] Executor created");

    let _node = ReactiveFollowGap::new(&executor)?;
    //ReactiveFollowGap::new(&executor)?;

    println!("[DEBUG] Starting executor spin...");
    println!("[DEBUG] Waiting for /scan messages...\n");
    executor.spin(SpinOptions::default()).first_error()
}
