// ================================
// src/control.rs - 제어 관련 모든 기능 (하나의 파일)
// ================================
//use crate::config::Config;
use crate::Config;

// Pure Pursuit 제어
pub struct PurePursuitController {
    config: Config,
}

impl PurePursuitController {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    /// Pure Pursuit 알고리즘을 사용하여 조향각을 계산합니다
    pub fn calculate_steering(&self, target_angle: f32, lookahead_distance: f32) -> f32 {
        let target_point = self.calculate_target_point(target_angle, lookahead_distance);
        self.pure_pursuit_steering(target_point, lookahead_distance)
    }

    /// 목표점의 좌표를 계산합니다
    fn calculate_target_point(&self, angle: f32, distance: f32) -> (f32, f32) {
        let x = distance * angle.cos();
        let y = distance * angle.sin();
        (x, y)
    }

    /// Pure Pursuit 조향각을 계산합니다
    fn pure_pursuit_steering(&self, target_point: (f32, f32), _lookahead_distance: f32) -> f32 {
        let (x, y) = target_point;

        // 차량 후축 기준으로 좌표 변환
        let adjusted_x = x + self.config.lidar_to_rear;
        let lookahead_angle = y.atan2(adjusted_x);
        let adjusted_lookahead = (adjusted_x.powi(2) + y.powi(2)).sqrt();

        // Pure Pursuit 공식
        2.0 * self.config.wheel_base * lookahead_angle.sin() / adjusted_lookahead
    }
}

// 속도 제어
pub struct SpeedController {
    config: Config,
}

impl SpeedController {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    /// 조향각에 따른 적응적 속도를 계산합니다
    pub fn calculate_adaptive_speed(&self, steering_angle: f32) -> f32 {
        let steering_ratio = (steering_angle.abs() / self.config.max_steering_rad).clamp(0.0, 1.0);

        // 선형 보간으로 속도 계산
        self.lerp(self.config.max_speed, self.config.min_speed, steering_ratio)
    }

    /// 적응적 전방 주시 거리를 계산합니다
    pub fn calculate_adaptive_lookahead(&self, base_distance: f32, steering_angle: f32) -> f32 {
        let steering_ratio = (steering_angle.abs() / self.config.max_steering_rad).clamp(0.0, 1.0);

        // 조향각이 클수록 전방 주시 거리를 줄임
        self.lerp(base_distance, base_distance * 0.05, steering_ratio.sqrt())
    }

    /// 선형 보간 함수
    fn lerp(&self, a: f32, b: f32, t: f32) -> f32 {
        a + t * (b - a)
    }
}
