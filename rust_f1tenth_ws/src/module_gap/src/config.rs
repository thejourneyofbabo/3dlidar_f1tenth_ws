// ================================
// src/config.rs - 모든 상수와 설정
// ================================
use std::f32::consts::PI;

#[derive(Debug, Clone)] // Clone trait 추가!
pub struct Config {
    // LiDAR 관련 설정
    pub max_range: f32,
    pub min_range: f32,
    pub roi_angle_deg: f32,
    pub min_gap_range: f32,

    // 차량 물리적 속성
    pub vehicle_width: f32,
    pub lidar_to_rear: f32,
    pub wheel_base: f32,

    // 제어 파라미터
    pub min_speed: f32,
    pub max_speed: f32,
    pub max_steering_rad: f32,
    pub ema_alpha: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            max_range: 5.0,
            min_range: 0.3,
            vehicle_width: 0.4,
            roi_angle_deg: 69.0,
            min_gap_range: 1.5,
            ema_alpha: 0.7,
            lidar_to_rear: 0.27,
            wheel_base: 0.32,
            min_speed: 0.5,
            max_speed: 3.5,
            max_steering_rad: PI / 4.0,
        }
    }
}
