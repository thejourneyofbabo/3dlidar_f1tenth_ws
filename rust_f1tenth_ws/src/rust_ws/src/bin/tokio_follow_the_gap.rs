use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{f32::consts::PI, sync::Arc};
use tokio::sync::{mpsc, Mutex, RwLock};
use tokio::time::{timeout, Duration};

// Compile-time constants
const MAX_RANGE: f32 = 5.0;
const MIN_RANGE: f32 = 0.3;
const VEHICLE_WIDTH: f32 = 0.4;
const ROI_ANGLE_DEG: f32 = 67.0;
const ROI_ANGLE_RAD: f32 = ROI_ANGLE_DEG * PI / 180.0;
const MIN_GAP_RANGE: f32 = 1.8;
const EMA_ALPHA: f32 = 0.7;

// Vehicle constants
const LIDAR_TO_REAR: f32 = 0.27;
const WHEEL_BASE: f32 = 0.32;

// Speed lookup table
//const SPEED_TABLE: [(f32, f32); 4] = [(5.0, 4.0), (10.0, 2.5), (15.0, 1.2), (f32::MAX, 0.8)];
const SPEED_TABLE: [(f32, f32); 6] = [
    (3.0, 4.5),
    (5.0, 4.0),
    (7.5, 3.0),
    (10.0, 2.5),
    (15.0, 1.2),
    (f32::MAX, 0.8),
];

// Lookahead multipliers - reduced for tighter cornering
const LOOKAHEAD_TABLE: [(f32, f32); 5] = [
    (3.0, 1.0),
    (5.0, 0.8),      // was 1.0
    (15.0, 0.5),     // was 0.7
    (30.0, 0.3),     // was 0.5
    (f32::MAX, 0.2), // was 0.3
];

#[allow(dead_code)]
struct ReactiveFollowGap {
    scan_receiver: mpsc::Receiver<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<RwLock<Option<usize>>>,
    // Reusable buffer for processed ranges
    processed_ranges_buffer: Arc<Mutex<Vec<f32>>>,
    // Keep subscription alive
    _scan_subscription: Subscription<LaserScan>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;
        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        // Create channel for async processing
        let (tx, rx) = mpsc::channel::<LaserScan>(10);

        // Subscribe to scan topic
        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                // Non-blocking send to async processor
                if let Err(e) = tx.try_send(msg) {
                    eprintln!("Failed to send scan message: {}", e);
                }
            })?;

        // Pre-allocate buffer
        let processed_ranges_buffer = Arc::new(Mutex::new(Vec::with_capacity(1080)));

        Ok(Self {
            scan_receiver: rx,
            drive_publisher,
            previous_best_point: Arc::new(RwLock::new(None)),
            processed_ranges_buffer,
            _scan_subscription: scan_subscription,
        })
    }

    async fn run(&mut self) {
        while let Some(msg) = self.scan_receiver.recv().await {
            // Skip old messages and process only the latest
            let mut latest_msg = msg;
            while let Ok(newer_msg) = self.scan_receiver.try_recv() {
                latest_msg = newer_msg;
            }

            // Process with timeout to ensure real-time constraints
            match timeout(Duration::from_millis(10), self.process_scan(latest_msg)).await {
                Ok(Ok(())) => {}
                Ok(Err(e)) => eprintln!("Error processing scan: {}", e),
                Err(_) => eprintln!("Scan processing timeout!"),
            }
        }
    }

    async fn process_scan(&self, msg: LaserScan) -> Result<(), Error> {
        // Process lidar data with reusable buffer
        let processed_ranges = self.process_lidar(&msg).await;

        // Find max gap
        let (max_gap_start, max_gap_end) = Self::find_max_gap(&msg, &processed_ranges);

        // Find best point with async read lock
        let prev_best = self.previous_best_point.read().await;
        let best_point_idx =
            Self::find_best_point(&processed_ranges, max_gap_start, max_gap_end, *prev_best);
        drop(prev_best);

        // Update best point with write lock
        {
            let mut prev_best = self.previous_best_point.write().await;
            *prev_best = Some(best_point_idx);
        }

        // Calculate control
        let (steering_angle, drive_speed) = Self::vehicle_control(&msg, best_point_idx);

        // Publish drive command
        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        self.drive_publisher.publish(&drive_msg)?;

        Ok(())
    }

    async fn process_lidar(&self, msg: &LaserScan) -> Vec<f32> {
        let mut buffer = self.processed_ranges_buffer.lock().await;

        // Reuse buffer memory
        buffer.clear();
        buffer.extend_from_slice(&msg.ranges);

        let mut min_dist = MAX_RANGE;
        let mut min_idx = 0;

        // Process ranges
        for (i, range) in buffer.iter_mut().enumerate() {
            if *range > MAX_RANGE || range.is_nan() || range.is_infinite() {
                *range = MAX_RANGE;
            } else if *range < min_dist && *range > 0.0 {
                min_dist = *range;
                min_idx = i;
            }
        }

        // Apply safety bubble
        if min_dist < MIN_RANGE {
            let bubble_radius = ((VEHICLE_WIDTH / 2.0) / min_dist) / msg.angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = std::cmp::min(min_idx + bubble_size, buffer.len() - 1);

            for i in start_idx..=end_idx {
                buffer[i] = 0.0;
            }
        }

        buffer.clone()
    }

    fn find_max_gap(msg: &LaserScan, ranges: &[f32]) -> (usize, usize) {
        // Calculate ROI indices using pre-computed constant
        let roi_angle_steps = (ROI_ANGLE_RAD / msg.angle_increment) as usize;
        let mid_lidar_idx = ranges.len() / 2;
        let roi_idx_start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let roi_idx_end = (mid_lidar_idx + roi_angle_steps).min(ranges.len() - 1);

        let mut max_gap_size = 0;
        let mut max_gap_start = roi_idx_start;
        let mut max_gap_end = roi_idx_start;
        let mut gap_start = roi_idx_start;
        let mut in_gap = false;

        for i in roi_idx_start..=roi_idx_end {
            let is_free = ranges[i] > MIN_GAP_RANGE;

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
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }

        if max_gap_size == 0 {
            println!("Warning: No gap found!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        (max_gap_start, max_gap_end)
    }

    fn find_best_point(
        ranges: &[f32],
        gap_start: usize,
        gap_end: usize,
        previous_best: Option<usize>,
    ) -> usize {
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

        // Apply EMA filter
        match previous_best {
            Some(prev) => (EMA_ALPHA * (best_point as f32) + (1.0 - EMA_ALPHA) * (prev as f32))
                .round() as usize,
            None => best_point,
        }
    }

    #[inline(always)]
    fn pure_pursuit(steer_ang_rad: f32, lookahead_dist: f32) -> f32 {
        let bestpoint_x = lookahead_dist * steer_ang_rad.cos();
        let bestpoint_y = lookahead_dist * steer_ang_rad.sin();

        let lookahead_angle = bestpoint_y.atan2(bestpoint_x + LIDAR_TO_REAR);
        let lookahead_rear = ((bestpoint_x + LIDAR_TO_REAR).powi(2) + bestpoint_y.powi(2)).sqrt();

        (2.0 * WHEEL_BASE * lookahead_angle.sin()).atan2(lookahead_rear)
    }

    fn vehicle_control(msg: &LaserScan, best_point: usize) -> (f32, f32) {
        let vehicle_center_idx = msg.ranges.len() / 2;
        let steer_ang_rad = (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let best_lookahead = msg.ranges[best_point].min(3.0);
        let steer_ang_deg = steer_ang_rad.abs() * 180.0 / PI;

        // Use lookup table for adaptive lookahead
        let adaptive_lookahead = LOOKAHEAD_TABLE
            .iter()
            .find(|(angle, _)| steer_ang_deg < *angle)
            .map(|(_, mult)| best_lookahead * mult)
            .unwrap_or(best_lookahead * 0.3);

        let pure_pursuit_steer = Self::pure_pursuit(steer_ang_rad, adaptive_lookahead);
        let steer_ang_deg = pure_pursuit_steer.abs() * 180.0 / PI;

        // Use lookup table for speed
        let drive_speed = SPEED_TABLE
            .iter()
            .find(|(angle, _)| steer_ang_deg < *angle)
            .map(|(_, speed)| *speed)
            .unwrap_or(0.8);

        println!(
            "Final Steer: {:.2}°, Driving Speed: {:.2} m/s",
            steer_ang_deg, drive_speed
        );

        (pure_pursuit_steer, drive_speed)
    }
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("F1Tenth Gap Follow Node with Rust (Tokio Optimized)");

    // Create ROS2 context and executor
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    // Create node
    let node = ReactiveFollowGap::new(&executor)?;

    // Spawn async processing task
    let processing_handle = tokio::spawn(async move {
        let mut node = node;
        node.run().await;
    });

    // Run ROS2 executor in main thread
    println!("Starting ROS2 executor...");

    // Use tokio's block_in_place to run the executor
    tokio::task::block_in_place(|| executor.spin(SpinOptions::default()).first_error())?;

    // Wait for processing task to complete
    processing_handle.await?;

    Ok(())
}
