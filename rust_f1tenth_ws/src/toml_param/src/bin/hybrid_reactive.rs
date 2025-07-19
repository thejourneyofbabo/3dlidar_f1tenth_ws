// src/bin/hybrid_reactive.rs

use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use serde::Deserialize;
use std::{
    env,
    f64::consts::PI,
    fs,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, SystemTime},
};

#[derive(Deserialize, Debug, Clone)]
pub struct VehicleParams {
    // LiDAR processing parameters
    pub max_range: f64,
    pub min_range: f64,
    pub min_gap_range: f64,
    // Vehicle physical parameters
    pub vehicle_width: f64,
    pub lidar_to_rear: f64,
    pub wheel_base: f64,
    // Control parameters
    pub min_speed: f64,
    pub max_speed: f64,
    pub max_steering_rad: f64,
    // Algorithm parameters
    pub roi_angle_deg: f64,
    pub ema_alpha: f64,
    // Topic names
    pub scan_topic: String,
    pub drive_topic: String,
    // Debug options
    pub debug_mode: bool,
    pub publish_debug_info: bool,
}

impl VehicleParams {
    fn load() -> Result<Self, Box<dyn std::error::Error>> {
        let config_path =
            env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string());
        let config_str = fs::read_to_string(&config_path)?;
        Ok(toml::from_str(&config_str)?)
    }
}

struct ParameterManager {
    params: Arc<Mutex<VehicleParams>>,
    config_path: String,
    last_modified: Arc<Mutex<SystemTime>>,
}

impl ParameterManager {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let config_path =
            env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string());
        let params = Arc::new(Mutex::new(VehicleParams::load()?));

        let metadata = fs::metadata(&config_path)?;
        let last_modified = Arc::new(Mutex::new(metadata.modified()?));

        println!("Parameter manager initialized with config: {}", config_path);

        Ok(Self {
            params,
            config_path,
            last_modified,
        })
    }

    fn check_and_reload(&self) -> Result<bool, Box<dyn std::error::Error>> {
        let metadata = fs::metadata(&self.config_path)?;
        let modified = metadata.modified()?;

        let mut last_modified_guard = self.last_modified.lock().unwrap();

        if modified > *last_modified_guard {
            match VehicleParams::load() {
                Ok(new_params) => {
                    let mut params_guard = self.params.lock().unwrap();
                    *params_guard = new_params;
                    *last_modified_guard = modified;
                    drop(params_guard);
                    drop(last_modified_guard);
                    println!("Parameters reloaded from file!");
                    return Ok(true);
                }
                Err(e) => {
                    eprintln!("Failed to parse config file: {}", e);
                    return Err(e);
                }
            }
        }
        Ok(false)
    }

    fn get_params(&self) -> Arc<Mutex<VehicleParams>> {
        self.params.clone()
    }

    fn start_file_watcher(&self) {
        let params = self.params.clone();
        let config_path = self.config_path.clone();
        let last_modified = self.last_modified.clone();

        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(500)); // Check every 500ms

                if let Ok(metadata) = fs::metadata(&config_path) {
                    if let Ok(modified) = metadata.modified() {
                        let mut last_modified_guard = last_modified.lock().unwrap();

                        if modified > *last_modified_guard {
                            match VehicleParams::load() {
                                Ok(new_params) => {
                                    let mut params_guard = params.lock().unwrap();
                                    *params_guard = new_params;
                                    *last_modified_guard = modified;
                                    drop(params_guard);
                                    drop(last_modified_guard);
                                    println!("Parameters hot-reloaded!");
                                }
                                Err(e) => {
                                    eprintln!("Hot-reload failed: {}", e);
                                }
                            }
                        }
                    }
                } else {
                    eprintln!("Config file not found: {}", config_path);
                }
            }
        });
    }
}

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
    params: Arc<Mutex<VehicleParams>>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor, params: Arc<Mutex<VehicleParams>>) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        let params_lock = params.lock().unwrap();
        if params_lock.debug_mode {
            println!("=== Initial Parameters ===");
            println!("{:#?}", *params_lock);
        }

        let drive_publisher =
            node.create_publisher::<AckermannDriveStamped>(&params_lock.drive_topic)?;
        let scan_topic = params_lock.scan_topic.clone();
        drop(params_lock);

        let publisher_clone = drive_publisher.clone();
        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();
        let params_clone = params.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>(&scan_topic, move |msg: LaserScan| {
                if let Err(e) =
                    Self::lidar_callback(&msg, &publisher_clone, &prev_best_clone, &params_clone)
                {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
            params,
        })
    }

    fn process_lidar(msg: &LaserScan, params: &VehicleParams) -> Vec<f32> {
        let mut ranges = msg.ranges.clone();
        let mut min_dist = params.max_range as f32;
        let mut min_idx = 0;

        for (i, range) in ranges.iter_mut().enumerate() {
            if *range > params.max_range as f32 || range.is_nan() || range.is_infinite() {
                *range = params.max_range as f32;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        if min_dist < params.min_range as f32 {
            let bubble_radius =
                ((params.vehicle_width as f32 / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, ranges.len() - 1);

            for i in start_idx..=end_idx {
                ranges[i] = 0.0
            }
        }

        ranges
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32], params: &VehicleParams) -> (usize, usize) {
        let roi_angle_rad = (params.roi_angle_deg as f32) * (PI as f32) / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize;
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > params.min_gap_range as f32;

            if is_free && !in_gap {
                gap_start = i;
                in_gap = true;
            } else if !is_free && in_gap {
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
            if params.debug_mode {
                println!("Warning!: No gap found!!");
            }
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
        params: &VehicleParams,
    ) -> usize {
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

    fn vehicle_control(msg: &LaserScan, best_point: usize, params: &VehicleParams) -> (f32, f32) {
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let best_lookahead = msg.ranges[best_point].min(3.0);
        let lerp = |a: f32, b: f32, t: f32| a + t * (b - a);

        let steering_ratio =
            (steer_ang_rad.abs() / (params.max_steering_rad as f32)).clamp(0.0, 1.0);
        let adaptive_lookahead = lerp(best_lookahead, best_lookahead * 0.05, steering_ratio.sqrt());

        let pure_pursuit_steer = Self::pure_pursuit(&steer_ang_rad, adaptive_lookahead, params);
        let steering_ratio =
            (pure_pursuit_steer.abs() / (params.max_steering_rad as f32)).clamp(0.0, 1.0);

        let drive_speed = lerp(
            params.max_speed as f32,
            params.min_speed as f32,
            steering_ratio,
        );

        if params.publish_debug_info {
            println!(
                "adaptive_lookahead: {}, steering_ratio: {:.6}, final_speed: {}, max_speed: {}",
                adaptive_lookahead,
                steering_ratio.sqrt(),
                drive_speed,
                params.max_speed
            );
        }

        (pure_pursuit_steer, drive_speed)
    }

    fn lidar_callback(
        msg: &LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        params: &Arc<Mutex<VehicleParams>>,
    ) -> Result<(), Error> {
        let params_guard = params.lock().unwrap();
        let current_params = params_guard.clone();
        drop(params_guard);

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

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node (Hybrid TOML + Hot Reload)");

    // Initialize parameter manager
    let param_manager = ParameterManager::new().expect("Failed to load TOML config");

    // Start file watcher thread
    param_manager.start_file_watcher();

    // Initialize ROS2 node
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor, param_manager.get_params())?;

    println!("Hot reload enabled - edit your TOML file and save to see changes!");
    println!(
        "Config file: {}",
        env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string())
    );

    executor.spin(SpinOptions::default()).first_error()
}
