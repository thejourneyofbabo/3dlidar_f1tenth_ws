use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::{f32::consts::PI, usize};

struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,
}

impl ReactiveFollowGap {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        let publisher_clone = drive_publisher.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(msg) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
        })
    }

    fn process_lidar(msg: &LaserScan) -> Vec<LaserScan> {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        const MAX_LIDAR_RANGE: f32 = 3.0;

        let filtered_points: Vec<f32> = msg
            .ranges
            .iter()
            .map(|&range| {
                if range <= MAX_LIDAR_RANGE {
                    range
                } else {
                    MAX_LIDAR_RANGE
                }
            })
            .collect();

        let mut processed_scan = msg.clone();
        processed_scan.ranges = filtered_points;

        vec![processed_scan]
    }

    fn find_max_gap(msg: &LaserScan) -> (usize, usize) {
        // Return the start index & end index of the max gap in free_space_ranges
        let mut roi_angle_deg = 67.0; // degree
        let mut roi_angle_rad = roi_angle_deg * PI / 180.0;
        let roi_angle_steps = (roi_angle_rad / msg.angle_increment) as usize; // ROI angle steps
                                                                              // for left & right
        let mid_lidar_idx = ((msg.angle_max - msg.angle_min) / msg.angle_increment) as usize;

        let (start_idx, end_idx) = (
            mid_lidar_idx - roi_angle_steps,
            mid_lidar_idx + roi_angle_steps,
        );

        let mut points = Vec::with_capacity(end_idx - start_idx + 1);

        for i in start_idx..=end_idx {}
    }

    fn find_best_point() {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
    }

    fn lidar_callback(msg: LaserScan) -> Result<(), Error> {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        let lidar_points = Self::process_lidar(&msg);

        // Todo
        // Find closest point to LiDAR
        let (max_gap_start, max_gap_end) = Self::find_max_gap(&lidar_points);

        // Eliminate all points inside 'bubble' (set them to zero)

        // Find max length gap

        // Find the best point in the gap

        // Publish Drive message
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
