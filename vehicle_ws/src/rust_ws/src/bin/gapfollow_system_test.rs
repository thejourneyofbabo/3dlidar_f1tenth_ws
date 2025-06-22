use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    fs::OpenOptions,
    io::Write,
    sync::{Arc, Mutex},
    time::Instant,
};

#[derive(Debug, Clone, Default)]
struct PerformanceMetrics {
    preprocess_time_ms: f64,
    find_gap_time_ms: f64,
    find_best_point_time_ms: f64,
    control_time_ms: f64,
    total_time_ms: f64,
    scan_size: usize,
    frequency_hz: f64,
}

#[derive(Debug, Default)]
struct RunningStats {
    avg_total_time_ms: f64,
    max_total_time_ms: f64,
    min_total_time_ms: f64,
    sample_count: u64,
}

impl RunningStats {
    fn new() -> Self {
        Self {
            min_total_time_ms: f64::MAX,
            ..Default::default()
        }
    }

    fn update(&mut self, current_time: f64) {
        self.sample_count += 1;

        // Update running average
        let alpha = 1.0 / self.sample_count as f64;
        self.avg_total_time_ms = (1.0 - alpha) * self.avg_total_time_ms + alpha * current_time;

        // Update min/max
        self.max_total_time_ms = self.max_total_time_ms.max(current_time);
        self.min_total_time_ms = self.min_total_time_ms.min(current_time);
    }
}

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
    performance_stats: Arc<Mutex<RunningStats>>,
    last_callback_time: Arc<Mutex<Option<Instant>>>,
    performance_file: Arc<Mutex<Option<std::fs::File>>>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>(
            "/drive"
                .durability(QoSDurabilityPolicy::Volatile)
                .keep_last(5),
        )?;
        let publisher_clone = drive_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();

        // Performance tracking initialization
        let performance_stats = Arc::new(Mutex::new(RunningStats::new()));
        let perf_stats_clone = performance_stats.clone();

        let last_callback_time = Arc::new(Mutex::new(None));
        let last_time_clone = last_callback_time.clone();

        // Initialize performance log file
        let performance_file = Arc::new(Mutex::new(
            OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open("performance_log_rust.csv")
                .map(|mut file| {
                    writeln!(file, "timestamp,total_time_ms,preprocess_ms,find_gap_ms,find_best_point_ms,control_ms,scan_size,frequency_hz").ok();
                    file
                })
                .ok()
        ));
        let perf_file_clone = performance_file.clone();

        let scan_subscription = node.create_subscription::<LaserScan, _>(
            "/scan"
                .best_effort()
                .durability(QoSDurabilityPolicy::Volatile)
                .keep_last(3),
            move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(
                    msg,
                    &publisher_clone,
                    &prev_best_clone,
                    &perf_stats_clone,
                    &last_time_clone,
                    &perf_file_clone,
                ) {
                    eprintln!("Error during scan process: {}", e);
                }
            },
        )?;

        println!("ReactiveFollowGap node initialized with performance monitoring");

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
            performance_stats,
            last_callback_time,
            performance_file,
        })
    }

    fn process_lidar(msg: &LaserScan) -> (Vec<f32>, f64) {
        let start = Instant::now();

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        let max_range: f32 = 5.0;
        let min_range: f32 = 0.3;
        let vehicle_width: f32 = 0.4;

        let mut ranges = msg.ranges.clone();
        let mut min_dist = max_range;
        let mut min_idx = 0;

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

        let elapsed = start.elapsed().as_nanos() as f64 / 1_000_000.0; // Convert to milliseconds
        (ranges, elapsed)
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32]) -> ((usize, usize), f64) {
        let start = Instant::now();

        // Set Region of Interest(ROI)
        let roi_angle_deg = 67.0; // degree
        let roi_angle_rad = roi_angle_deg * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize; // ROI angle steps
                                                                              // for left & right
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        // Return the start index & end index of the max gap in free_space_ranges
        let min_range = 1.5;
        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > min_range;

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
            println!("Warning: No gap found!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        let elapsed = start.elapsed().as_nanos() as f64 / 1_000_000.0; // Convert to milliseconds
        ((max_gap_start, max_gap_end), elapsed)
    }

    fn find_best_point(
        ranges: &[f32],
        gap_start: usize,
        gap_end: usize,
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> (usize, f64) {
        let start = Instant::now();

        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
        let alpha = 0.6;
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

        let elapsed = start.elapsed().as_nanos() as f64 / 1_000_000.0; // Convert to milliseconds
        (ema_best, elapsed)
    }

    fn pure_pursuit(steer_ang_rad: &f32, lookahead_dist: f32) -> f32 {
        let lidar_to_rear = 0.27;
        let wheel_base = 0.32;
        let bestpoint_x = lookahead_dist * f32::cos(*steer_ang_rad);
        let bestpoint_y = lookahead_dist * f32::sin(*steer_ang_rad);

        let lookahead_angle = f32::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
        let lookahead_rear = f32::sqrt((bestpoint_x + lidar_to_rear).powi(2) + bestpoint_y.powi(2));

        // Final Pure Pursuit Angle
        f32::atan2(2.0 * wheel_base * lookahead_angle.sin(), lookahead_rear)
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> ((f32, f32), f64) {
        let start = Instant::now();

        // Calculate Steering-Angle & Speed
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad: f32 =
            (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let best_lookahead = msg.ranges[best_point].min(3.0);
        let steer_ang_deg = steer_ang_rad.abs() * 180.0 / PI;

        let adaptive_lookahead = match steer_ang_deg {
            n if n < 5.0 => best_lookahead * 1.0,
            n if n < 15.0 => best_lookahead * 0.7,
            n if n < 30.0 => best_lookahead * 0.5,
            _ => best_lookahead * 0.3,
        };

        let pure_pursuit_steer = Self::pure_pursuit(&steer_ang_rad, adaptive_lookahead);
        let steer_ang_deg = pure_pursuit_steer.abs() * 180.0 / PI;

        // Fast Speed
        let drive_speed = match steer_ang_deg {
            n if n < 5.0 => 4.0,
            n if n < 10.0 => 2.5,
            n if n < 15.0 => 1.2,
            _ => 0.8,
        };

        let elapsed = start.elapsed().as_nanos() as f64 / 1_000_000.0; // Convert to milliseconds
        ((pure_pursuit_steer, drive_speed), elapsed)
    }

    fn print_performance_stats(stats: &RunningStats, current_metrics: &PerformanceMetrics) {
        if stats.sample_count == 0 {
            return;
        }

        println!("\n=== PERFORMANCE STATISTICS ===");
        println!("Average Processing Time: {:.3} ms", stats.avg_total_time_ms);
        println!("Min Processing Time: {:.3} ms", stats.min_total_time_ms);
        println!("Max Processing Time: {:.3} ms", stats.max_total_time_ms);
        println!(
            "Average Frequency: {:.1} Hz",
            1000.0 / stats.avg_total_time_ms
        );
        println!("Scan Size: {} points", current_metrics.scan_size);
        println!("Samples: {}", stats.sample_count);
        println!("--- Breakdown ---");
        println!(
            "Preprocess: {:.3} ms ({:.1}%)",
            current_metrics.preprocess_time_ms,
            (current_metrics.preprocess_time_ms / stats.avg_total_time_ms) * 100.0
        );
        println!(
            "Find Gap: {:.3} ms ({:.1}%)",
            current_metrics.find_gap_time_ms,
            (current_metrics.find_gap_time_ms / stats.avg_total_time_ms) * 100.0
        );
        println!(
            "Find Best Point: {:.3} ms ({:.1}%)",
            current_metrics.find_best_point_time_ms,
            (current_metrics.find_best_point_time_ms / stats.avg_total_time_ms) * 100.0
        );
        println!(
            "Control: {:.3} ms ({:.1}%)",
            current_metrics.control_time_ms,
            (current_metrics.control_time_ms / stats.avg_total_time_ms) * 100.0
        );
        println!("===============================\n");
    }

    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        performance_stats: &Arc<Mutex<RunningStats>>,
        last_callback_time: &Arc<Mutex<Option<Instant>>>,
        performance_file: &Arc<Mutex<Option<std::fs::File>>>,
    ) -> Result<(), Error> {
        let total_start = Instant::now();
        let mut current_metrics = PerformanceMetrics::default();

        // Calculate frequency
        let current_time = Instant::now();
        {
            let mut last_time_lock = last_callback_time.lock().unwrap();
            if let Some(last_time) = *last_time_lock {
                let time_diff = current_time.duration_since(last_time);
                current_metrics.frequency_hz = 1.0 / time_diff.as_secs_f64();
            }
            *last_time_lock = Some(current_time);
        }

        current_metrics.scan_size = msg.ranges.len();

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        let (processed_ranges, preprocess_time) = Self::process_lidar(&msg);
        current_metrics.preprocess_time_ms = preprocess_time;

        // Find max length gap
        let ((max_gap_start, max_gap_end), find_gap_time) =
            Self::find_max_gap(&msg, &processed_ranges);
        current_metrics.find_gap_time_ms = find_gap_time;

        // Find the best point in the gap
        let (best_point_idx, find_best_time) = Self::find_best_point(
            &processed_ranges,
            max_gap_start,
            max_gap_end,
            previous_best_point,
        );
        current_metrics.find_best_point_time_ms = find_best_time;

        let ((steering_angle, drive_speed), control_time) =
            Self::vehicle_control(&msg, best_point_idx);
        current_metrics.control_time_ms = control_time;

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        drive_publisher.publish(&drive_msg)?;

        // Calculate total time and update stats
        current_metrics.total_time_ms = total_start.elapsed().as_nanos() as f64 / 1_000_000.0;

        // Update performance statistics
        {
            let mut stats = performance_stats.lock().unwrap();
            stats.update(current_metrics.total_time_ms);

            // Print stats every 1000 samples (similar to C++ timer-based approach)
            if stats.sample_count % 1000 == 0 {
                Self::print_performance_stats(&stats, &current_metrics);
            }
        }

        // Log to CSV file
        if let Ok(mut file_lock) = performance_file.lock() {
            if let Some(ref mut file) = *file_lock {
                let timestamp = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis();

                writeln!(
                    file,
                    "{},{:.3},{:.3},{:.3},{:.3},{:.3},{},{:.1}",
                    timestamp,
                    current_metrics.total_time_ms,
                    current_metrics.preprocess_time_ms,
                    current_metrics.find_gap_time_ms,
                    current_metrics.find_best_point_time_ms,
                    current_metrics.control_time_ms,
                    current_metrics.scan_size,
                    current_metrics.frequency_hz
                )
                .ok();
                file.flush().ok();
            }
        }

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("ReactiveFollowGap node with Rust - Performance Analysis Edition");
    println!("Performance monitoring: Enabled");
    println!("Log file: performance_log_rust.csv");
    println!("Stats printed every 1000 samples");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
