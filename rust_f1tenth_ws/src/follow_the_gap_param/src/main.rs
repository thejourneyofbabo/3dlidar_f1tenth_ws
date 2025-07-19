use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use follow_the_gap_param::vehicle_setup::param_config::{ParameterManager, VehicleParams};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    env,
    f64::consts::PI,
    sync::{Arc, Mutex},
};

/// Reactive Follow-the-Gap algorithm implementation
#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
    params: Arc<Mutex<VehicleParams>>,
}

impl ReactiveFollowGap {
    /// Create new ReactiveFollowGap node with given parameters
    fn new(executor: &Executor, params: Arc<Mutex<VehicleParams>>) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        // Display initial parameters if debug mode is enabled
        let params_lock = params.lock().unwrap();
        if params_lock.debug_mode {
            println!("=== Initial Parameters ===");
            println!("{:#?}", *params_lock);
        }

        // Create publishers and subscribers
        let drive_publisher =
            node.create_publisher::<AckermannDriveStamped>(&params_lock.drive_topic)?;
        let scan_topic = params_lock.scan_topic.clone();
        drop(params_lock);

        // Setup callback data sharing
        let publisher_clone = drive_publisher.clone();
        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();
        let params_clone = params.clone();

        // Create LiDAR subscription with callback
        let scan_subscription =
            node.create_subscription::<LaserScan, _>(&scan_topic, move |msg: LaserScan| {
                if let Err(e) =
                    Self::lidar_callback(&msg, &publisher_clone, &prev_best_clone, &params_clone)
                {
                    eprintln!("Error during scan processing: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
            params,
        })
    }

    /// Process LiDAR data: filter invalid ranges and create safety bubble
    fn process_lidar(msg: &LaserScan, params: &VehicleParams) -> Vec<f32> {
        let mut ranges = msg.ranges.clone();
        let mut min_dist = params.max_range as f32;
        let mut min_idx = 0;

        // Filter invalid ranges and find closest obstacle
        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > params.max_range as f32 || range.is_nan() || range.is_infinite() {
                *range = params.max_range as f32;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        // Create safety bubble around closest obstacle
        if min_dist < params.min_range as f32 {
            let bubble_radius =
                ((params.vehicle_width as f32 / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, ranges.len() - 1);

            // Zero out ranges within safety bubble
            for i in start_idx..=end_idx {
                ranges[i] = 0.0
            }
        }

        ranges
    }

    /// Find the largest gap within the region of interest (ROI)
    fn find_max_gap(msg: &LaserScan, ranges: &[f32], params: &VehicleParams) -> (usize, usize) {
        // Calculate ROI boundaries
        let roi_angle_rad = (params.roi_angle_deg as f32) * (PI as f32) / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize;
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        // Gap detection variables
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        // Scan through ROI to find gaps
        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > params.min_gap_range as f32;

            if is_free && !in_gap {
                // Start of new gap
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

        // Handle gap extending to end of ROI
        if in_gap {
            let gap_size = roi_idx_end - gap_start + 1;
            if gap_size > max_gap_size {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }

        // Fallback if no gap found
        if max_gap_size == 0 {
            if params.debug_mode {
                println!("Warning: No gap found!");
            }
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        (max_gap_start, max_gap_end)
    }

    /// Find the best target point within the gap using weighted average and EMA filtering
    fn find_best_point(
        ranges: &[f32],
        gap_start: usize,
        gap_end: usize,
        previous_best: &Arc<Mutex<Option<usize>>>,
        params: &VehicleParams,
    ) -> usize {
        let mut weighted_sum = 0.0;
        let mut weight_total = 0.0;

        // Calculate weighted center of gap (farther points have higher weight)
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

        // Apply Exponential Moving Average (EMA) filter for smooth steering
        let ema_best = {
            let mut prev_lock = previous_best.lock().unwrap();
            let ema_result = match *prev_lock {
                Some(prev) => ((params.ema_alpha as f32) * (best_point as f32)
                    + (1.0 - params.ema_alpha as f32) * (prev as f32))
                    .round() as usize,
                None => best_point,
            };

            *prev_lock = Some(ema_result);
            ema_result
        };

        if params.debug_mode {
            println!("Best Point = {}", ema_best);
        }

        ema_best
    }

    /// Pure pursuit algorithm for calculating steering angle
    fn pure_pursuit(steer_ang_rad: &f32, lookahead_dist: f32, params: &VehicleParams) -> f32 {
        let bestpoint_x = lookahead_dist * f32::cos(*steer_ang_rad);
        let bestpoint_y = lookahead_dist * f32::sin(*steer_ang_rad);

        let lookahead_angle = f32::atan2(bestpoint_y, bestpoint_x + params.lidar_to_rear as f32);
        let lookahead_rear =
            f32::sqrt((bestpoint_x + params.lidar_to_rear as f32).powi(2) + bestpoint_y.powi(2));

        f32::atan2(
            2.0 * (params.wheel_base as f32) * lookahead_angle.sin(),
            lookahead_rear,
        )
    }

    /// Calculate steering angle and speed based on best target point
    fn vehicle_control(msg: &LaserScan, best_point: usize, params: &VehicleParams) -> (f32, f32) {
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let best_lookahead = msg.ranges[best_point].min(3.0);
        let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);

        // Calculate adaptive lookahead distance
        let steering_ratio =
            (steer_ang_rad.abs() / (params.max_steering_rad as f32)).clamp(0.0, 1.0);
        let adaptive_lookahead = lerp(best_lookahead, best_lookahead * 0.05, steering_ratio.sqrt());

        // Apply pure pursuit for final steering calculation
        let pure_pursuit_steer = Self::pure_pursuit(&steer_ang_rad, adaptive_lookahead, params);
        let steering_ratio =
            (pure_pursuit_steer.abs() / (params.max_steering_rad as f32)).clamp(0.0, 1.0);

        // Speed control: slower for sharper turns
        let drive_speed = lerp(
            params.max_speed as f32,
            params.min_speed as f32,
            steering_ratio,
        );

        if params.publish_debug_info {
            println!(
                "adaptive_lookahead: {:.3}, steering_ratio: {:.6}, speed: {:.3}, max_speed: {:.3}",
                adaptive_lookahead,
                steering_ratio.sqrt(),
                drive_speed,
                params.max_speed
            );
        }

        (pure_pursuit_steer, drive_speed)
    }

    /// Main LiDAR callback function - processes scan and publishes drive commands
    fn lidar_callback(
        msg: &LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        params: &Arc<Mutex<VehicleParams>>,
    ) -> Result<(), Error> {
        // Get current parameters (hot-reload safe)
        let current_params = params.lock().unwrap().clone();

        // Execute Follow-the-Gap algorithm pipeline
        let processed_ranges = Self::process_lidar(msg, &current_params);
        let (max_gap_start, max_gap_end) =
            Self::find_max_gap(msg, &processed_ranges, &current_params);
        let best_point_idx = Self::find_best_point(
            &processed_ranges,
            max_gap_start,
            max_gap_end,
            previous_best_point,
            &current_params,
        );

        let (steering_angle, drive_speed) =
            Self::vehicle_control(msg, best_point_idx, &current_params);

        // Publish drive command
        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node (Hybrid TOML + Hot Reload)");

    // Initialize parameter manager with hot-reload capability
    let param_manager = ParameterManager::new().expect("Failed to load TOML configuration");
    param_manager.start_file_watcher();

    // Initialize ROS2 node and executor
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor, param_manager.get_params())?;

    println!("Hot reload enabled - edit your TOML file and save to see changes!");
    println!(
        "Config file: {}",
        env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string())
    );

    // Start main execution loop
    executor.spin(SpinOptions::default()).first_error()
}
