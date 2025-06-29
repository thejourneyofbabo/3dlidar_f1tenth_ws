// ================================
// src/node.rs - ROS2 노드 관리
// ================================
use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::sync::{Arc, Mutex};

use crate::{
    config::Config,
    control::{PurePursuitController, SpeedController},
    lidar::{GapFinder, LidarProcessor},
};

#[allow(dead_code)]
pub struct ReactiveFollowGap {
    scan_subscription: Subscription<LaserScan>,
    drive_publisher: Publisher<AckermannDriveStamped>,

    // 알고리즘 컴포넌트들 - Arc로 감싸서 공유 가능하게 만듦
    lidar_processor: Arc<LidarProcessor>,
    gap_finder: Arc<GapFinder>,
    pursuit_controller: Arc<PurePursuitController>,
    speed_controller: Arc<SpeedController>,

    // 상태 관리
    previous_best_point: Arc<Mutex<Option<usize>>>,
    config: Config,
}

impl ReactiveFollowGap {
    pub fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("follow_the_gap_node")?;
        let config = Config::default();

        let drive_publisher = node.create_publisher::<AckermannDriveStamped>("/drive")?;
        let publisher_clone = drive_publisher.clone();

        let previous_best_point = Arc::new(Mutex::new(None));
        let prev_best_clone = previous_best_point.clone();

        // 컴포넌트 초기화 - Arc로 감싸서 공유 가능하게 만듦
        let lidar_processor = Arc::new(LidarProcessor::new(config.clone()));
        let gap_finder = Arc::new(GapFinder::new(config.clone()));
        let pursuit_controller = Arc::new(PurePursuitController::new(config.clone()));
        let speed_controller = Arc::new(SpeedController::new(config.clone()));

        // 클로저에서 사용할 복사본들 생성
        let lidar_processor_clone = lidar_processor.clone();
        let gap_finder_clone = gap_finder.clone();
        let pursuit_controller_clone = pursuit_controller.clone();
        let speed_controller_clone = speed_controller.clone();

        let scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(
                    msg,
                    &publisher_clone,
                    &prev_best_clone,
                    &lidar_processor_clone,
                    &gap_finder_clone,
                    &pursuit_controller_clone,
                    &speed_controller_clone,
                ) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
            lidar_processor,
            gap_finder,
            pursuit_controller,
            speed_controller,
            previous_best_point,
            config,
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn lidar_callback(
        msg: LaserScan,
        drive_publisher: &Publisher<AckermannDriveStamped>,
        previous_best_point: &Arc<Mutex<Option<usize>>>,
        lidar_processor: &Arc<LidarProcessor>,
        gap_finder: &Arc<GapFinder>,
        pursuit_controller: &Arc<PurePursuitController>,
        speed_controller: &Arc<SpeedController>,
    ) -> Result<(), Error> {
        // 1. LiDAR 데이터 전처리
        let processed_ranges = lidar_processor.process(&msg);

        // 2. 최대 갭 찾기
        let max_gap = gap_finder.find_max_gap(&msg, &processed_ranges);

        // 3. 최적 포인트 결정
        let best_point_idx =
            gap_finder.find_best_point(&processed_ranges, &max_gap, previous_best_point);

        // 4. 조향각 계산
        let steering_angle = Self::calculate_vehicle_control(
            &msg,
            best_point_idx,
            pursuit_controller,
            speed_controller,
        )?;

        // 5. 드라이브 메시지 발행
        Self::publish_drive_command(drive_publisher, steering_angle)?;

        Ok(())
    }

    fn calculate_vehicle_control(
        msg: &LaserScan,
        best_point: usize,
        pursuit_controller: &Arc<PurePursuitController>,
        speed_controller: &Arc<SpeedController>,
    ) -> Result<(f32, f32), Error> {
        let vehicle_center_idx = msg.ranges.len() / 2;
        let target_angle = (best_point as f32 - vehicle_center_idx as f32) * msg.angle_increment;

        let base_lookahead = msg.ranges[best_point].min(3.0);
        let adaptive_lookahead =
            speed_controller.calculate_adaptive_lookahead(base_lookahead, target_angle);

        let steering_angle =
            pursuit_controller.calculate_steering(target_angle, adaptive_lookahead);
        let drive_speed = speed_controller.calculate_adaptive_speed(steering_angle);

        // 디버그 출력
        Self::print_debug_info(adaptive_lookahead, steering_angle, drive_speed);

        Ok((steering_angle, drive_speed))
    }

    fn publish_drive_command(
        publisher: &Publisher<AckermannDriveStamped>,
        control: (f32, f32),
    ) -> Result<(), Error> {
        let (steering_angle, drive_speed) = control;

        let mut drive_msg = AckermannDriveStamped::default();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        publisher.publish(&drive_msg)?;
        Ok(())
    }

    fn print_debug_info(lookahead: f32, steering: f32, speed: f32) {
        println!("=== DEBUG ===");
        println!("adaptive_lookahead: {:.3}", lookahead);
        println!("steering_angle: {:.6}", steering);
        println!("final_speed: {:.3}", speed);
        println!("=============");
    }
}
