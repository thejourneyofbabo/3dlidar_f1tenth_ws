use ackermann_msgs::msg::AckermannDriveStamped;
use rclrs::*;
use std::{thread, time::Duration};

struct F1tenthBasicDrive {
    ackermann_pub: Publisher<AckermannDriveStamped>,
}

impl F1tenthBasicDrive {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("f1tenth_rust_basic_drive")?;
        let ackermann_pub = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        Ok(Self { ackermann_pub })
    }

    fn start(self) {
        thread::spawn(move || loop {
            //self.drive_forward(5.0);
            //self.stop_vehicle(2.0);
            //self.drive_backward(5.0);
            //self.stop_vehicle(2.0);
            self.drive_left(2.0);
            self.drive_right(2.0);
        });
    }

    fn drive_forward(&self, duration_sec: f64) {
        println!("Forward for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(1.0, 0.3);
            self.ackermann_pub.publish(msg).unwrap();
            thread::sleep(Duration::from_millis(50));
        }
    }

    fn drive_backward(&self, duration_sec: f64) {
        println!("Backward for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32;

        for _ in 0..iterations {
            let msg = Self::create_drive_message(-1.0, 0.0);
            self.ackermann_pub.publish(msg).unwrap();
            thread::sleep(Duration::from_millis(50));
        }
    }

    fn stop_vehicle(&self, duration_sec: f64) {
        println!("Stop for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32;

        for _ in 0..iterations {
            let msg = Self::create_drive_message(0.0, 0.0);
            self.ackermann_pub.publish(msg).unwrap();
            thread::sleep(Duration::from_millis(50));
        }
    }

    fn drive_left(&self, duration_sec: f64) {
        println!("Left for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(1.0, 0.1);
            self.ackermann_pub.publish(msg).unwrap();
            thread::sleep(Duration::from_millis(50));
        }
    }

    fn drive_right(&self, duration_sec: f64) {
        println!("Right for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(1.0, -0.1);
            self.ackermann_pub.publish(msg).unwrap();
            thread::sleep(Duration::from_millis(50));
        }
    }

    fn create_drive_message(speed: f32, steering_angle: f32) -> AckermannDriveStamped {
        let mut msg = AckermannDriveStamped::default();
        msg.drive.speed = speed;
        msg.drive.steering_angle = steering_angle;
        msg
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    F1tenthBasicDrive::new(&executor)?.start();
    executor.spin(SpinOptions::default()).first_error()
}
