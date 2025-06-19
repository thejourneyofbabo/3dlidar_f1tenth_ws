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
        let max_range: f32 = 5.0;
        let min_range: f32 = 0.3;
        let vehicle_width: f32 = 0.4;

        let mut ranges = msg.ranges.clone();
        let mut min_dist = max_range;
        let mut min_idx = 0;

        //let ranges: Vec<f32> = msg
        //    .ranges
        //    .iter()
        //    .map(|&range| if range <= max_range { range } else { max_range })
        //    .collect();

        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > max_range || range.is_nan() || range.is_infinite() {
                *range = max_range;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        if min_dist < min_range {
            let bubble_radius = ((vehicle_width / 2.0) / min_dist) / msg.angle_increment;
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
        //println!(
        //    "roi_idx_start: {}\nroi_idx_end:{}",
        //    roi_idx_start, roi_idx_end
        //);

        // Return the start index & end index of the max gap in free_space_ranges
        let min_range = 1.5;
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > min_range;
            //let was_free = i > roi_idx_start && ranges[i - 1] > min_range;

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
            println!("Warngin!: No gap found!!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        //let mid_point = ranges.len() / 2;
        //println!(
        //    "max_gap_left:{}\nmax_gap_right:{}",
        //    mid_point - max_gap_start,
        //    max_gap_end - mid_point
        //);

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
        let alpha = 0.7;
        let mut weighted_sum = 0.0;
        let mut weight_total = 0.0;

        for i in gap_start..=gap_end {
            let weight = ranges[i];
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
                Some(prev) => {
                    (alpha * (best_point as f32) + (1.0 - alpha) * (prev as f32)).round() as usize
                }
                None => best_point,
            };

            *prev_lock = Some(ema_result);
            ema_result
        };
        println!("Best Point = {}", ema_best);

        ema_best
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> (f32, f32) {
        // Calculate Steering-Angle & Speed
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;
        let steer_ang_rad = steer_ang_rad.clamp(-0.4, 0.4);
        let steer_ang_deg = steer_ang_rad.abs() * 180.0 / PI;

        // Normal Speed
        //let drive_speed = match steer_ang_deg {
        //    n if n < 5.0 => 2.0,
        //    n if n < 10.0 => 1.5,
        //    n if n < 15.0 => 1.2,
        //    _ => 0.8,
        //};

        // Fast Speed
        let drive_speed = match steer_ang_deg {
            n if n < 5.0 => 4.0,
            n if n < 10.0 => 2.5,
            n if n < 15.0 => 1.2,
            _ => 0.8,
        };

        (steer_ang_rad, drive_speed)
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
    ) -> Result<(), Error> {
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
        //drive_msg.drive.speed = 0.0;

        drive_publisher.publish(&drive_msg)?;

        // Publish Drive message
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    //ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
