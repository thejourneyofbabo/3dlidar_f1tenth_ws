// src/vehicle_params.rs
use rclrs::*;
use std::f64::consts::PI;
use std::sync::Arc;

#[derive(Debug, Clone)]
pub struct VehicleParams {
    // LiDAR 처리 파라미터
    pub max_range: f64,
    pub min_range: f64,
    pub min_gap_range: f64,

    // 차량 물리 파라미터
    pub vehicle_width: f64,
    pub lidar_to_rear: f64,
    pub wheel_base: f64,

    // 제어 파라미터
    pub min_speed: f64,
    pub max_speed: f64,
    pub max_steering_rad: f64,

    // 알고리즘 파라미터
    pub roi_angle_deg: f64,
    pub ema_alpha: f64,

    // 토픽 이름
    pub scan_topic: Arc<str>,
    pub drive_topic: Arc<str>,

    // 디버그 옵션
    pub debug_mode: bool,
    pub publish_debug_info: bool,
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

impl VehicleParams {
    /// ROS 2 파라미터에서 로드 (YAML 파일 지원)
    pub fn from_ros_params(node: &Node) -> Result<Self, RclrsError> {
        let mut params = Self::default();

        // 각 파라미터를 ROS 2 파라미터 시스템에서 로드
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
}
