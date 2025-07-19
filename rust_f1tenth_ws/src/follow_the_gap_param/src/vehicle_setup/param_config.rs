// param_config.rs

use anyhow::Result;
use serde::Deserialize;
use std::{
    env, fs,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

/// Vehicle configuration parameters loaded from TOML file
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
    /// Load parameters from TOML configuration file
    pub fn load() -> Result<Self, Box<dyn std::error::Error>> {
        let config_path =
            env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string());
        let config_str = fs::read_to_string(&config_path)?;
        Ok(toml::from_str(&config_str)?)
    }
}

/// Simplified parameter manager with hot-reload capability
pub struct ParameterManager {
    params: Arc<Mutex<VehicleParams>>,
    config_path: String,
}

impl ParameterManager {
    /// Create new parameter manager and load initial configuration
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let config_path =
            env::var("CONFIG_PATH").unwrap_or_else(|_| "./vehicle_param.toml".to_string());
        let params = Arc::new(Mutex::new(VehicleParams::load()?));

        println!("Parameter manager initialized with config: {}", config_path);

        Ok(Self {
            params,
            config_path,
        })
    }

    /// Get thread-safe reference to parameters
    pub fn get_params(&self) -> Arc<Mutex<VehicleParams>> {
        self.params.clone()
    }

    /// Start background file watcher thread for hot-reload functionality
    pub fn start_file_watcher(&self) {
        let params = self.params.clone();
        let config_path = self.config_path.clone();
        let mut last_modified = fs::metadata(&config_path)
            .and_then(|m| m.modified())
            .unwrap_or(std::time::SystemTime::UNIX_EPOCH);

        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(500)); // Check every 500ms

                // Chain-based file change detection and parameter reload
                let reload_result = fs::metadata(&config_path)
                    .and_then(|metadata| metadata.modified())
                    .map(|modified| {
                        if modified > last_modified {
                            VehicleParams::load()
                                .map(|new_params| {
                                    if let Ok(mut guard) = params.lock() {
                                        *guard = new_params;
                                        last_modified = modified;
                                        println!("Parameters hot-reloaded!");
                                    }
                                })
                                .unwrap_or_else(|e| eprintln!("Hot-reload failed: {}", e));
                        }
                    });

                // Ignore errors (file not found, etc. are normal during startup)
                let _ = reload_result;
            }
        });
    }
}
