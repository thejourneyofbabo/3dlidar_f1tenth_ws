use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

// Define mission states
#[derive(Debug, Clone, Copy, PartialEq)]
enum MissionType {
    MissionA, // Camera mission
    MissionB, // Lidar mission (in narrow walls)
    MissionC, // Odometry mission
}

impl MissionType {
    fn as_str(&self) -> &'static str {
        match self {
            MissionType::MissionA => "MISSION_A",
            MissionType::MissionB => "MISSION_B",
            MissionType::MissionC => "MISSION_C",
        }
    }
}

// Configuration constants
const CLOSE_THRESHOLD: f32 = 0.5; // < 0.5m is close
const MEDIUM_THRESHOLD: f32 = 0.7; // < 0.7m is medium, >= 0.7m is far
const B_DETECTION_THRESHOLD: i32 = 5; // Consecutive detections needed for A->B
const C_DETECTION_THRESHOLD: i32 = 10; // Consecutive detections needed for B->C
const TARGET_OFFSET_X: f32 = 0.8; // Forward offset for Mission C
const TARGET_OFFSET_Y: f32 = 2.5; // Right offset for Mission C

// Shared state for mission transitions
#[derive(Debug)]
struct MissionState {
    current_mission: MissionType,
    current_mission_str: String,
    consecutive_b_detections: i32,
    consecutive_c_detections: i32,
}

impl Default for MissionState {
    fn default() -> Self {
        Self {
            current_mission: MissionType::MissionA,
            current_mission_str: String::new(),
            consecutive_b_detections: 0,
            consecutive_c_detections: 0,
        }
    }
}

struct MissionLauncher {
    _scan_subscription: Subscription<LaserScan>,
    mission_publisher: Publisher<StringMsg>,
    mission_state: Arc<Mutex<MissionState>>,
}

impl MissionLauncher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("mission_launcher")?;

        let mission_publisher = node.create_publisher::<StringMsg>("current_mission")?;
        let publisher_clone = mission_publisher.clone();

        let mission_state = Arc::new(Mutex::new(MissionState::default()));
        let state_clone = mission_state.clone();

        let _scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(msg, &publisher_clone, &state_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        // Publish initial mission state
        let mut initial_msg = StringMsg::default();
        initial_msg.data = MissionType::MissionA.as_str().to_string();
        mission_publisher.publish(&initial_msg).ok();

        println!("Mission Launcher initialized - Starting in MISSION_A");

        Ok(Self {
            _scan_subscription,
            mission_publisher,
            mission_state,
        })
    }

    fn filter_valid_ranges(ranges: &[f32]) -> Vec<f32> {
        ranges
            .iter()
            .filter(|&&range| range.is_finite() && range > 0.0)
            .copied()
            .collect()
    }

    fn calculate_percentages(ranges: &[f32]) -> (f32, f32, f32) {
        let total = ranges.len() as f32;

        let close_count = ranges.iter().filter(|&&r| r < CLOSE_THRESHOLD).count() as f32;
        let medium_count = ranges
            .iter()
            .filter(|&&r| r >= CLOSE_THRESHOLD && r < MEDIUM_THRESHOLD)
            .count() as f32;
        let far_count = total - close_count - medium_count;

        (
            (close_count / total) * 100.0,
            (medium_count / total) * 100.0,
            (far_count / total) * 100.0,
        )
    }

    fn analyze_scan_regions(scan_msg: &LaserScan) -> (f32, f32, f32, f32, f32, f32) {
        let ranges = &scan_msg.ranges;
        let center_index = ranges.len() / 2;
        let angle_width = ranges.len() / 6; // ~30 degrees on each side

        // Collect ranges for each region
        let front_ranges: Vec<f32> = ((center_index.saturating_sub(angle_width))
            ..=(center_index + angle_width).min(ranges.len() - 1))
            .filter_map(|i| {
                let range = ranges[i];
                if range.is_finite() && range > 0.0 {
                    Some(range)
                } else {
                    None
                }
            })
            .collect();

        let left_ranges: Vec<f32> = (0..(angle_width * 2).min(ranges.len()))
            .filter_map(|i| {
                let range = ranges[i];
                if range.is_finite() && range > 0.0 {
                    Some(range)
                } else {
                    None
                }
            })
            .collect();

        let right_ranges: Vec<f32> = ((ranges.len().saturating_sub(angle_width * 2))..ranges.len())
            .filter_map(|i| {
                let range = ranges[i];
                if range.is_finite() && range > 0.0 {
                    Some(range)
                } else {
                    None
                }
            })
            .collect();

        // Calculate means
        let front_mean = if front_ranges.is_empty() {
            0.0
        } else {
            front_ranges.iter().sum::<f32>() / front_ranges.len() as f32
        };
        let left_mean = if left_ranges.is_empty() {
            0.0
        } else {
            left_ranges.iter().sum::<f32>() / left_ranges.len() as f32
        };
        let right_mean = if right_ranges.is_empty() {
            0.0
        } else {
            right_ranges.iter().sum::<f32>() / right_ranges.len() as f32
        };

        // Calculate close percentages
        let front_close_percent = if front_ranges.is_empty() {
            0.0
        } else {
            (front_ranges
                .iter()
                .filter(|&&r| r < CLOSE_THRESHOLD)
                .count() as f32
                / front_ranges.len() as f32)
                * 100.0
        };
        let left_close_percent = if left_ranges.is_empty() {
            0.0
        } else {
            (left_ranges.iter().filter(|&&r| r < CLOSE_THRESHOLD).count() as f32
                / left_ranges.len() as f32)
                * 100.0
        };
        let right_close_percent = if right_ranges.is_empty() {
            0.0
        } else {
            (right_ranges
                .iter()
                .filter(|&&r| r < CLOSE_THRESHOLD)
                .count() as f32
                / right_ranges.len() as f32)
                * 100.0
        };

        (
            front_mean,
            left_mean,
            right_mean,
            front_close_percent,
            left_close_percent,
            right_close_percent,
        )
    }

    fn detect_narrow_passage(
        front_mean: f32,
        left_mean: f32,
        right_mean: f32,
        front_close_percent: f32,
        left_close_percent: f32,
        right_close_percent: f32,
    ) -> bool {
        // Check if we're between walls
        let walls_detected = left_close_percent > 35.0 && right_close_percent > 35.0;
        let clear_path_ahead = front_close_percent < 30.0 && front_mean > MEDIUM_THRESHOLD;

        // Check for approaching narrow passage
        let approaching_narrow = (left_mean < 1.2 && right_mean < 1.2)
            && (front_mean > MEDIUM_THRESHOLD * 1.2)
            && ((left_mean - right_mean).abs() < 0.4);

        (walls_detected && clear_path_ahead) || approaching_narrow
    }

    fn detect_rear_disparity(scan_msg: &LaserScan) -> bool {
        let disparity_threshold = 1.0;
        let degree_to_index = scan_msg.ranges.len() as f32 / 270.0;
        let rear_center_index = (180.0 * degree_to_index) as usize;
        let rear_range = (10.0 * degree_to_index) as usize;

        let start_idx = rear_center_index.saturating_sub(rear_range);
        let end_idx = (rear_center_index + rear_range).min(scan_msg.ranges.len() - 1);

        for i in (start_idx + 1)..=end_idx {
            if i < scan_msg.ranges.len() {
                let prev_range = scan_msg.ranges[i - 1];
                let curr_range = scan_msg.ranges[i];

                if prev_range.is_finite()
                    && curr_range.is_finite()
                    && (prev_range - curr_range).abs() > disparity_threshold
                {
                    let angle_deg = i as f32 / degree_to_index;
                    println!(
                        "Rear disparity detected at {:.1}°: {:.2}m jump",
                        angle_deg,
                        (prev_range - curr_range).abs()
                    );
                    return true;
                }
            }
        }
        false
    }

    fn detect_corridor_exit(
        scan_msg: &LaserScan,
        medium_percent: f32,
        far_percent: f32,
        front_mean: f32,
        front_open_percent: f32,
    ) -> bool {
        // Check for rear disparity (exit detection)
        let rear_disparity = Self::detect_rear_disparity(scan_msg);

        // Multi-criteria exit detection
        let general_exit = (medium_percent + far_percent > 65.0)
            && (front_open_percent > 80.0)
            && (front_mean > MEDIUM_THRESHOLD * 1.5);

        general_exit || rear_disparity
    }

    fn publish_mission(
        mission_name: &str,
        publisher: &Publisher<StringMsg>,
        state: &Arc<Mutex<MissionState>>,
    ) -> Result<(), Error> {
        let mut mission_state = state.lock().unwrap();

        if mission_name != mission_state.current_mission_str {
            let mut message = StringMsg::default();
            message.data = mission_name.to_string();
            publisher.publish(&message)?;
            mission_state.current_mission_str = mission_name.to_string();

            println!("Mission changed to: {}", mission_name);
        }
        Ok(())
    }

    fn lidar_callback(
        scan_msg: LaserScan,
        mission_publisher: &Publisher<StringMsg>,
        mission_state: &Arc<Mutex<MissionState>>,
    ) -> Result<(), Error> {
        if scan_msg.ranges.is_empty() {
            eprintln!("Empty scan message received");
            return Ok(());
        }

        // Filter valid ranges
        let valid_ranges = Self::filter_valid_ranges(&scan_msg.ranges);
        if valid_ranges.is_empty() {
            eprintln!("No valid range measurements");
            return Ok(());
        }

        // Calculate overall statistics
        let mean_range = valid_ranges.iter().sum::<f32>() / valid_ranges.len() as f32;
        let (close_percent, medium_percent, far_percent) =
            Self::calculate_percentages(&valid_ranges);

        // Analyze scan regions
        let (
            front_mean,
            left_mean,
            right_mean,
            front_close_percent,
            left_close_percent,
            right_close_percent,
        ) = Self::analyze_scan_regions(&scan_msg);

        // Mission state transitions
        {
            let mut state = mission_state.lock().unwrap();

            match state.current_mission {
                MissionType::MissionA => {
                    if Self::detect_narrow_passage(
                        front_mean,
                        left_mean,
                        right_mean,
                        front_close_percent,
                        left_close_percent,
                        right_close_percent,
                    ) {
                        state.consecutive_b_detections += 1;
                        println!(
                            "Narrow passage detected ({}/{})",
                            state.consecutive_b_detections, B_DETECTION_THRESHOLD
                        );

                        if state.consecutive_b_detections >= B_DETECTION_THRESHOLD {
                            println!("Transitioning: MISSION_A -> MISSION_B");
                            state.current_mission = MissionType::MissionB;
                            state.consecutive_b_detections = 0;
                            drop(state); // Release lock before calling publish
                            Self::publish_mission("MISSION_B", mission_publisher, mission_state)?;
                        }
                    } else if state.consecutive_b_detections > 0 {
                        state.consecutive_b_detections -= 1;
                    }
                }

                MissionType::MissionB => {
                    // Calculate front open percentage for corridor exit detection
                    let center_index = scan_msg.ranges.len() / 2;
                    let angle_width = scan_msg.ranges.len() / 6;

                    let front_ranges: Vec<f32> = ((center_index.saturating_sub(angle_width))
                        ..=(center_index + angle_width).min(scan_msg.ranges.len() - 1))
                        .filter_map(|i| {
                            let range = scan_msg.ranges[i];
                            if range.is_finite() && range > 0.0 {
                                Some(range)
                            } else {
                                None
                            }
                        })
                        .collect();

                    let front_open_percent = if front_ranges.is_empty() {
                        0.0
                    } else {
                        (front_ranges
                            .iter()
                            .filter(|&&r| r > MEDIUM_THRESHOLD)
                            .count() as f32
                            / front_ranges.len() as f32)
                            * 100.0
                    };

                    if Self::detect_corridor_exit(
                        &scan_msg,
                        medium_percent,
                        far_percent,
                        front_mean,
                        front_open_percent,
                    ) {
                        state.consecutive_c_detections += 1;
                        println!(
                            "Corridor exit detected ({}/{})",
                            state.consecutive_c_detections, C_DETECTION_THRESHOLD
                        );

                        if state.consecutive_c_detections >= C_DETECTION_THRESHOLD {
                            println!("*** CORRIDOR EXIT CONFIRMED ***");
                            println!("Transitioning: MISSION_B -> MISSION_C");
                            state.current_mission = MissionType::MissionC;
                            state.consecutive_c_detections = 0;
                            drop(state); // Release lock before calling publish
                            Self::publish_mission("MISSION_C", mission_publisher, mission_state)?;
                        }
                    } else if state.consecutive_c_detections > 0 {
                        state.consecutive_c_detections -= 1;
                    }
                }

                MissionType::MissionC => {
                    // Stay in Mission C
                }
            }
        }

        // Logging
        let current_mission_str = {
            let state = mission_state.lock().unwrap();
            match state.current_mission {
                MissionType::MissionA => "A",
                MissionType::MissionB => "B",
                MissionType::MissionC => "C",
            }
        };

        println!(
            "Scan: {} points, Mean: {:.2}m, Close: {:.1}%, Medium: {:.1}%, Far: {:.1}% | Mission: {}",
            valid_ranges.len(),
            mean_range,
            close_percent,
            medium_percent,
            far_percent,
            current_mission_str
        );

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("Rust Mission Launcher Node");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _launcher = MissionLauncher::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
