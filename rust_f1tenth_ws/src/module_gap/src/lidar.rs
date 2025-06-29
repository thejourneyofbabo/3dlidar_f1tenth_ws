// ================================
// src/lidar.rs - LiDAR 관련 모든 기능 (하나의 파일)
// ================================
use crate::config::Config;
use sensor_msgs::msg::LaserScan;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

// LiDAR 데이터 전처리
pub struct LidarProcessor {
    config: Config,
}

impl LidarProcessor {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    /// LiDAR 스캔 데이터를 전처리합니다.
    pub fn process(&self, msg: &LaserScan) -> Vec<f32> {
        let mut ranges = msg.ranges.clone();

        // 1단계: 유효하지 않은 값들 처리
        self.clean_invalid_ranges(&mut ranges);

        // 2단계: Safety Bubble 생성
        if let Some(closest_obstacle) = self.find_closest_obstacle(&ranges) {
            self.create_safety_bubble(&mut ranges, closest_obstacle, msg.angle_increment);
        }

        ranges
    }

    fn clean_invalid_ranges(&self, ranges: &mut [f32]) {
        for range in ranges.iter_mut() {
            if *range > self.config.max_range || range.is_nan() || range.is_infinite() {
                *range = self.config.max_range;
            }
        }
    }

    fn find_closest_obstacle(&self, ranges: &[f32]) -> Option<(usize, f32)> {
        ranges
            .iter()
            .enumerate()
            .filter(|(_, &range)| range > 0.0 && range < self.config.max_range)
            .min_by(|(_, &a), (_, &b)| a.partial_cmp(&b).unwrap())
            .map(|(idx, &range)| (idx, range))
    }

    fn create_safety_bubble(
        &self,
        ranges: &mut [f32],
        closest: (usize, f32),
        angle_increment: f32,
    ) {
        let (min_idx, min_dist) = closest;

        if min_dist < self.config.min_range {
            let bubble_radius = ((self.config.vehicle_width / 2.0) / min_dist) / angle_increment;
            let bubble_size = bubble_radius as usize;

            let start_idx = min_idx.saturating_sub(bubble_size);
            let end_idx = (min_idx + bubble_size).min(ranges.len() - 1);

            for range in ranges.iter_mut().take(end_idx + 1).skip(start_idx) {
                *range = 0.0;
            }
        }
    }
}

// 갭 찾기 관련
#[derive(Debug, Clone)]
pub struct Gap {
    pub start_idx: usize,
    pub end_idx: usize,
    pub size: usize,
}

pub struct GapFinder {
    config: Config,
}

impl GapFinder {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn find_max_gap(&self, msg: &LaserScan, ranges: &[f32]) -> Gap {
        let roi = self.calculate_roi(ranges.len(), msg.angle_increment);
        self.find_largest_gap_in_roi(ranges, roi)
    }

    pub fn find_best_point(
        &self,
        ranges: &[f32],
        gap: &Gap,
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> usize {
        let raw_best = self.calculate_weighted_center(ranges, gap);
        self.apply_ema_filter(raw_best, previous_best)
    }

    fn calculate_roi(&self, total_ranges: usize, angle_increment: f32) -> (usize, usize) {
        let roi_angle_rad = self.config.roi_angle_deg * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / angle_increment) as usize;
        let mid_lidar_idx = total_ranges / 2;

        let start = mid_lidar_idx.saturating_sub(roi_angle_steps);
        let end = (mid_lidar_idx + roi_angle_steps).min(total_ranges - 1);

        (start, end)
    }

    fn find_largest_gap_in_roi(&self, ranges: &[f32], roi: (usize, usize)) -> Gap {
        let (roi_start, roi_end) = roi;
        let mut max_gap = Gap {
            start_idx: roi_start,
            end_idx: roi_start,
            size: 0,
        };
        let mut current_gap_start = roi_start;
        let mut in_gap = false;

        for i in roi_start..=roi_end {
            let is_free = ranges[i] > self.config.min_gap_range;

            match (is_free, in_gap) {
                (true, false) => {
                    current_gap_start = i;
                    in_gap = true;
                }
                (false, true) => {
                    let gap_size = i - current_gap_start;
                    if gap_size > max_gap.size {
                        max_gap = Gap {
                            start_idx: current_gap_start,
                            end_idx: i - 1,
                            size: gap_size,
                        };
                    }
                    in_gap = false;
                }
                _ => {}
            }
        }

        if in_gap {
            let gap_size = roi_end - current_gap_start + 1;
            if gap_size > max_gap.size {
                max_gap = Gap {
                    start_idx: current_gap_start,
                    end_idx: roi_end,
                    size: gap_size,
                };
            }
        }

        if max_gap.size == 0 {
            eprintln!("Warning: No gap found!");
            let center = (roi_start + roi_end) / 2;
            max_gap = Gap {
                start_idx: center,
                end_idx: center,
                size: 1,
            };
        }

        max_gap
    }

    fn calculate_weighted_center(&self, ranges: &[f32], gap: &Gap) -> usize {
        let mut weighted_sum = 0.0;
        let mut weight_total = 0.0;

        for i in gap.start_idx..=gap.end_idx {
            let weight = ranges[i].powi(1);
            weighted_sum += i as f32 * weight;
            weight_total += weight;
        }

        if weight_total > 0.0 {
            (weighted_sum / weight_total) as usize
        } else {
            (gap.start_idx + gap.end_idx) / 2
        }
    }

    fn apply_ema_filter(
        &self,
        current_best: usize,
        previous_best: &Arc<Mutex<Option<usize>>>,
    ) -> usize {
        let mut prev_lock = previous_best.lock().unwrap();
        let filtered_best = match *prev_lock {
            Some(prev) => (self.config.ema_alpha * (current_best as f32)
                + (1.0 - self.config.ema_alpha) * (prev as f32))
                .round() as usize,
            None => current_best,
        };

        *prev_lock = Some(filtered_best);
        filtered_best
    }
}
