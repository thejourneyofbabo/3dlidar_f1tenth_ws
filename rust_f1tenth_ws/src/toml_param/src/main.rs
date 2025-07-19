use serde::Deserialize;
use std::{env, fs};

#[derive(Deserialize, Debug, Clone)]
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
    pub scan_topic: String,
    pub drive_topic: String,

    // 디버그 옵션
    pub debug_mode: bool,
    pub publish_debug_info: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config_path =
        env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string());
    let config_str = fs::read_to_string(&config_path)?;
    //let config_str = fs::read_to_string("./vehicle_param.toml")?;
    let config: VehicleParams = toml::from_str(&config_str)?;

    println!("Max range: {}", config.max_range);

    Ok(())
}
