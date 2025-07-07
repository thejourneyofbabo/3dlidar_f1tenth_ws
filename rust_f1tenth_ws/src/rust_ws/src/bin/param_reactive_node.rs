// follow_the_gap.rs

use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f64::consts::PI,
    sync::{Arc, Mutex},
};

// 파라미터를 저장할 구조체 (rclrs 호환 타입으로 변경)
#[derive(Debug, Clone)]
struct VehicleParams {
    // LiDAR 처리 파라미터 (f32 -> f64)
    max_range: f64,
    min_range: f64,
    min_gap_range: f64,

    // 차량 물리 파라미터 (f32 -> f64)
    vehicle_width: f64,
    lidar_to_rear: f64,
    wheel_base: f64,

    // 제어 파라미터 (f32 -> f64)
    min_speed: f64,
    max_speed: f64,
    max_steering_rad: f64,

    // 알고리즘 파라미터 (f32 -> f64)
    roi_angle_deg: f64,
    ema_alpha: f64,

    // 토픽 이름 (String -> Arc<str>)
    scan_topic: Arc<str>,
    drive_topic: Arc<str>,

    // 디버그 옵션
    debug_mode: bool,
    publish_debug_info: bool,
}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            max_range: 5.0,
            min_range: 0.3,
            min_gap_range: 1.5,
            vehicle_width: 0.4,
            lidar_to_rear: 0.27,
            wheel_base: 0.32,
            min_speed: 0.5,
            max_speed: 3.5,
            max_steering_rad: PI / 4.0,
            roi_angle_deg: 69.0,
            ema_alpha: 0.7,
            scan_topic: "/scan".into(),
            drive_topic: "/drive".into(),
            debug_mode: true,
            publish_debug_info: true,
        }
    }
}

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
    params: VehicleParams,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        // 파라미터 로드
        let params = Self::load_parameters(&node)?;

        // 파라미터 서비스 활성화 (undeclared parameters 허용)
        node.use_undeclared_parameters();

        if params.debug_mode {
            println!("=== Loaded Parameters ===");
            println!("{:#?}", params);
            println!("========================");
        }

        let drive_publisher =
            node.create_publisher::<AckermannDriveStamped>(&params.drive_topic)?;
        let publisher_clone = drive_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();
        let params_clone = params.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>(&params.scan_topic, move |msg: LaserScan| {
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

    fn load_parameters(node: &Node) -> Result<VehicleParams, RclrsError> {
        let mut params = VehicleParams::default();

        // 파라미터 선언 및 로드 (f64 타입 사용)
        if let Ok(param) = node
            .declare_parameter("max_range")
            .default(params.max_range)
            .mandatory()
        {
            params.max_range = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("min_range")
            .default(params.min_range)
            .mandatory()
        {
            params.min_range = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("min_gap_range")
            .default(params.min_gap_range)
            .mandatory()
        {
            params.min_gap_range = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("vehicle_width")
            .default(params.vehicle_width)
            .mandatory()
        {
            params.vehicle_width = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("lidar_to_rear")
            .default(params.lidar_to_rear)
            .mandatory()
        {
            params.lidar_to_rear = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("wheel_base")
            .default(params.wheel_base)
            .mandatory()
        {
            params.wheel_base = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("min_speed")
            .default(params.min_speed)
            .mandatory()
        {
            params.min_speed = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("max_speed")
            .default(params.max_speed)
            .mandatory()
        {
            params.max_speed = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("max_steering_rad")
            .default(params.max_steering_rad)
            .mandatory()
        {
            params.max_steering_rad = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("roi_angle_deg")
            .default(params.roi_angle_deg)
            .mandatory()
        {
            params.roi_angle_deg = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("ema_alpha")
            .default(params.ema_alpha)
            .mandatory()
        {
            params.ema_alpha = param.get();
        }

        // Arc<str> 타입 사용
        if let Ok(param) = node
            .declare_parameter("scan_topic")
            .default(params.scan_topic.clone())
            .mandatory()
        {
            params.scan_topic = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("drive_topic")
            .default(params.drive_topic.clone())
            .mandatory()
        {
            params.drive_topic = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("debug_mode")
            .default(params.debug_mode)
            .mandatory()
        {
            params.debug_mode = param.get();
        }

        if let Ok(param) = node
            .declare_parameter("publish_debug_info")
            .default(params.publish_debug_info)
            .mandatory()
        {
            params.publish_debug_info = param.get();
        }

        Ok(params)
    }

    fn process_lidar(msg: &LaserScan, params: &VehicleParams) -> Vec<f32> {
        let mut ranges = msg.ranges.clone();
        let mut min_dist = params.max_range as f32;
        println!("param_max_ranges: {}", min_dist);
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
            println!("=== DEBUG ===");
            println!("adaptive_lookahead: {}", adaptive_lookahead);
            println!("steering_ratio: {:.6}", steering_ratio.sqrt());
            println!("final_speed: {}", drive_speed);
            println!("=============");
        }

        (pure_pursuit_steer, drive_speed)
    }

    fn lidar_callback(
        msg: &LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        params: &VehicleParams,
    ) -> Result<(), Error> {
        let processed_ranges = Self::process_lidar(msg, params);
        let (max_gap_start, max_gap_end) = Self::find_max_gap(msg, &processed_ranges, params);
        let best_point_idx = Self::find_best_point(
            &processed_ranges,
            max_gap_start,
            max_gap_end,
            previous_best_point,
            params,
        );

        let (steering_angle, drive_speed) = Self::vehicle_control(msg, best_point_idx, params);

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust (Parameterized)");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
