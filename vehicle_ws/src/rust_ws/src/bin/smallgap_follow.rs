use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

// Configuration constants
const MAX_RANGE_CLIP: f32 = 5.0;
const MIN_RANGE_BUBBLE_TRIGGER: f32 = 0.8;
const VEHICLE_WIDTH: f32 = 0.5;
const ROI_ANGLE_DEG: f32 = 67.0;
const EMA_ALPHA: f32 = 0.7;
const SMOOTHING_WINDOW_SIZE: usize = 9;
const MAX_SPEED: f32 = 1.5;

#[allow(dead_code)]
pub struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
    previous_best_point: Arc<Mutex<Option<usize>>>,
}

impl ReactiveFollowGap {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;
        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        let previous_best_point = Arc::new(Mutex::new(None));
        let publisher_clone = drive_publisher.clone();
        let prev_best_clone = previous_best_point.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::process_scan(msg, &publisher_clone, &prev_best_clone) {
                    eprintln!("Scan processing error: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            previous_best_point,
        })
    }

    fn process_scan(
        msg: LaserScan,
        publisher: &Publisher<AckermannDriveStamped>,
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> Result<(), Error> {
        let processed_ranges = Self::preprocess_ranges(&msg);
        let best_point_idx = Self::find_best_point(&msg, &processed_ranges, previous_best);
        let (steering_angle, speed) = Self::calculate_control(&msg, best_point_idx);

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;

        publisher.publish(&drive_msg)?;
        Ok(())
    }

    fn preprocess_ranges(msg: &LaserScan) -> Vec<f32> {
        let cleaned_ranges: Vec<f32> = msg
            .ranges
            .iter()
            .map(|&r| {
                if r.is_nan() || r.is_infinite() || r <= 0.0 {
                    MAX_RANGE_CLIP
                } else {
                    r
                }
            })
            .collect();

        let scan_threshold = Self::calculate_dynamic_threshold(&cleaned_ranges);

        // Pad
        let padding = SMOOTHING_WINDOW_SIZE / 2;
        let data_size = cleaned_ranges.len();
        let mut padded = vec![0.0; data_size + padding * 2];

        for j in 0..padding {
            padded[j] = cleaned_ranges[0];
        }
        padded[padding..(padding + data_size)].copy_from_slice(&cleaned_ranges);
        for j in 0..padding {
            padded[padding + data_size + j] = cleaned_ranges[data_size - 1];
        }

        // Clipping
        for val in padded.iter_mut() {
            if *val > scan_threshold {
                *val = scan_threshold;
            }
        }

        // Smoothing
        let mut smoothed_ranges = vec![0.0; data_size];
        for i in 0..data_size {
            let sum: f32 = padded[i..(i + SMOOTHING_WINDOW_SIZE)].iter().sum();
            smoothed_ranges[i] = sum / SMOOTHING_WINDOW_SIZE as f32;
        }

        // Apply safety bubble
        Self::apply_safety_bubble(smoothed_ranges, msg.angle_increment)
    }

    fn calculate_dynamic_threshold(ranges: &[f32]) -> f32 {
        let min_range = ranges
            .iter()
            .filter(|r| r.is_finite() && **r > 0.0)
            .copied()
            .fold(MAX_RANGE_CLIP, f32::min);

        // Sigmoid parameters
        let a = 4.2;
        let b = 1.0;
        let k = 2.0;
        let x0 = 1.5;

        let threshold = b + (a - b) / (1.0 + (-k * (min_range - x0)).exp());
        threshold.clamp(1.0, MAX_RANGE_CLIP)

        //let threshold = b + (a - b) / (1.0 + (-k * (min_range - x0)).exp());
        //threshold.min(MAX_RANGE_CLIP).max(0.6)
    }

    fn apply_safety_bubble(mut ranges: Vec<f32>, angle_increment: f32) -> Vec<f32> {
        let mut min_dist = MAX_RANGE_CLIP;
        let mut min_idx = 0;

        for (i, &range) in ranges.iter().enumerate() {
            if range < min_dist && range > 0.0 {
                min_dist = range;
                min_idx = i;
            }
        }

        if min_dist < MIN_RANGE_BUBBLE_TRIGGER {
            let bubble_radius = ((VEHICLE_WIDTH / 2.0) / min_dist) / angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = (min_idx + bubble_size).min(ranges.len() - 1);

            for i in start_idx..=end_idx {
                ranges[i] = 0.0;
            }
        }

        ranges
    }

    fn find_best_point(
        msg: &LaserScan,
        ranges: &[f32],
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> usize {
        let roi_angle_rad = ROI_ANGLE_DEG * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize;
        let mid_idx = ranges.len() / 2;

        let roi_start = mid_idx.saturating_sub(roi_angle_steps);
        let roi_end = (mid_idx + roi_angle_steps).min(ranges.len() - 1);

        if roi_start >= roi_end {
            return mid_idx;
        }

        let mut max_dist = 0.0;
        for i in roi_start..=roi_end {
            if ranges[i] > max_dist {
                max_dist = ranges[i];
            }
        }

        let mut max_points = Vec::new();
        if max_dist > 0.0 {
            for i in roi_start..=roi_end {
                if (ranges[i] - max_dist).abs() < 0.001 {
                    max_points.push(i);
                }
            }
        }

        if max_points.is_empty() {
            return (roi_start + roi_end) / 2;
        }

        let best_center = Self::find_longest_segment_center(&max_points);
        Self::apply_ema_filter(best_center, previous_best)
    }

    fn find_longest_segment_center(points: &[usize]) -> usize {
        let mut max_start = points[0];
        let mut max_length = 1;
        let mut current_start = points[0];
        let mut current_length = 1;

        for i in 1..points.len() {
            if points[i] == points[i - 1] + 1 {
                current_length += 1;
            } else {
                if current_length > max_length {
                    max_length = current_length;
                    max_start = current_start;
                }
                current_start = points[i];
                current_length = 1;
            }
        }

        if current_length > max_length {
            max_start = current_start;
            max_length = current_length;
        }

        max_start + max_length / 2
    }

    fn apply_ema_filter(current_best: usize, previous_best: &Arc<Mutex<Option<usize>>>) -> usize {
        let mut prev_lock = previous_best.lock().unwrap();
        let filtered_best = match *prev_lock {
            Some(prev) => {
                (EMA_ALPHA * current_best as f32 + (1.0 - EMA_ALPHA) * prev as f32).round() as usize
            }
            None => current_best,
        };
        *prev_lock = Some(filtered_best);
        filtered_best
    }

    fn calculate_control(msg: &LaserScan, best_point_idx: usize) -> (f32, f32) {
        let steering_angle = msg.angle_min + msg.angle_increment * best_point_idx as f32;
        let steering_degree = steering_angle.abs() * 180.0 / PI;

        // Adaptive speed control based on steering
        let speed = (MAX_SPEED - steering_degree / 45.0).clamp(0.5, MAX_SPEED);

        println!(
            "Final Steer (deg): {:.2}\nDriving Speed: {:.2}",
            steering_degree, speed
        );
        (steering_angle, speed)
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node - Enhanced Sigmoid Version");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
