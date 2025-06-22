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
const EMA_ALPHA: f32 = 0.6;
const SMOOTHING_WINDOW_SIZE: usize = 9;

// Threshold points for dynamic range adjustment
const THRESHOLD_POINTS: [ThresholdPoint; 10] = [
    ThresholdPoint {
        range: 0.15,
        threshold: 0.4,
    },
    ThresholdPoint {
        range: 0.2,
        threshold: 0.8,
    },
    ThresholdPoint {
        range: 0.5,
        threshold: 1.4,
    },
    ThresholdPoint {
        range: 0.8,
        threshold: 1.8,
    },
    ThresholdPoint {
        range: 1.1,
        threshold: 2.4,
    },
    ThresholdPoint {
        range: 1.4,
        threshold: 2.8,
    },
    ThresholdPoint {
        range: 1.7,
        threshold: 3.2,
    },
    ThresholdPoint {
        range: 2.0,
        threshold: 3.6,
    },
    ThresholdPoint {
        range: 2.3,
        threshold: 3.9,
    },
    ThresholdPoint {
        range: 2.6,
        threshold: 4.2,
    },
];

#[derive(Clone, Copy)]
struct ThresholdPoint {
    range: f32,
    threshold: f32,
}

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
        let ranges = msg.ranges.clone();
        if ranges.is_empty() {
            return ranges;
        }

        // Calculate dynamic threshold
        let scan_threshold = Self::calculate_dynamic_threshold(&ranges);

        // Create padded array for smoothing
        let padding = SMOOTHING_WINDOW_SIZE / 2;
        let data_size = ranges.len();
        let mut padded = vec![0.0; data_size + padding * 2];

        // Add padding
        if data_size > 0 {
            // Left padding
            for j in 0..padding {
                padded[j] = ranges[0];
            }
            // Copy original data
            padded[padding..(padding + data_size)].copy_from_slice(&ranges);
            // Right padding
            for j in 0..padding {
                padded[padding + data_size + j] = ranges[data_size - 1];
            }
        }

        // Apply clipping to padded array
        for val in padded.iter_mut() {
            if val.is_nan() || val.is_infinite() || *val > scan_threshold {
                *val = scan_threshold;
            }
        }

        // Apply smoothing
        let mut smoothed_ranges = vec![0.0; data_size];
        for i in 0..data_size {
            let mut sum = 0.0;
            for j in 0..SMOOTHING_WINDOW_SIZE {
                sum += padded[i + j];
            }
            smoothed_ranges[i] = sum / SMOOTHING_WINDOW_SIZE as f32;
        }

        // Apply safety bubble
        Self::apply_safety_bubble(smoothed_ranges, msg.angle_increment)
    }

    fn calculate_dynamic_threshold(ranges: &[f32]) -> f32 {
        let min_range = ranges
            .iter()
            .filter(|&r| !r.is_nan() && !r.is_infinite() && *r > 0.0)
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .copied()
            .unwrap_or(MAX_RANGE_CLIP);

        // Linear interpolation
        if min_range <= THRESHOLD_POINTS[0].range {
            return THRESHOLD_POINTS[0].threshold;
        }

        if min_range >= THRESHOLD_POINTS[THRESHOLD_POINTS.len() - 1].range {
            return THRESHOLD_POINTS[THRESHOLD_POINTS.len() - 1].threshold;
        }

        // Find interpolation segment
        let mut i = 0;
        while i < THRESHOLD_POINTS.len() - 1 && min_range > THRESHOLD_POINTS[i].range {
            i += 1;
        }

        let p1 = &THRESHOLD_POINTS[i - 1];
        let p2 = &THRESHOLD_POINTS[i];
        let range_diff = p2.range - p1.range;
        let threshold_diff = p2.threshold - p1.threshold;
        let ratio = (min_range - p1.range) / range_diff;

        p1.threshold + (threshold_diff * ratio)
    }

    fn apply_safety_bubble(mut ranges: Vec<f32>, angle_increment: f32) -> Vec<f32> {
        // Find closest point
        let mut min_dist = MAX_RANGE_CLIP;
        let mut min_idx = 0;

        for (i, &range) in ranges.iter().enumerate() {
            if range < min_dist && range > 0.0 {
                min_dist = range;
                min_idx = i;
            }
        }

        // Apply bubble if too close
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

        // Find maximum distance in ROI
        let mut max_dist = 0.0;
        for i in roi_start..=roi_end {
            if ranges[i] > max_dist {
                max_dist = ranges[i];
            }
        }

        // Collect all points with maximum distance
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

        // Find longest contiguous segment
        let best_center = Self::find_longest_segment_center(&max_points);

        // Apply EMA filter
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

        // Check final segment
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

        let speed = if steering_degree <= 5.0 {
            1.2
        } else if steering_degree <= 10.0 {
            1.0
        } else if steering_degree <= 15.0 {
            0.8
        } else {
            0.5
        };

        println!(
            "Final Steer (deg): {:.2}\nDriving Speed: {:.2}",
            steering_degree, speed
        );
        (steering_angle, speed)
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node - Fixed Organized Version");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
