use ackermann_msgs::msg::AckermannDriveStamped;
use rclrs::*;
use std::{thread, time::Duration};

struct F1tenthBasicDrive {
    _ackermann_pub: Publisher<AckermannDriveStamped>,
}

impl F1tenthBasicDrive {
    /// Create a new F1tenthBasicDrive instance and start the driving thread
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("f1tenth_rust_basic_drive")?;
        let ackermann_pub = node.create_publisher::<AckermannDriveStamped>("/drive")?;

        // Start driving sequence in separate thread
        let publisher_clone = ackermann_pub.clone();
        thread::spawn(move || {
            Self::run_driving_sequence(publisher_clone);
        });

        Ok(Self {
            _ackermann_pub: ackermann_pub,
        })
    }

    /// Main driving sequence loop
    fn run_driving_sequence(publisher: Publisher<AckermannDriveStamped>) {
        loop {
            Self::drive_forward(&publisher, 5.0);
            Self::stop_vehicle(&publisher, 2.0);
            Self::drive_backward(&publisher, 3.0);
            Self::stop_vehicle(&publisher, 2.0);
        }
    }

    /// Drive forward for specified duration (in seconds)
    fn drive_forward(publisher: &Publisher<AckermannDriveStamped>, duration_sec: f64) {
        println!("Forward for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(1.0, 0.0);
            let _ = publisher.publish(&msg);
            thread::sleep(Duration::from_millis(50));
        }
    }

    /// Drive backward for specified duration (in seconds)
    fn drive_backward(publisher: &Publisher<AckermannDriveStamped>, duration_sec: f64) {
        println!("Backward for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(-0.5, 0.0);
            let _ = publisher.publish(&msg);
            thread::sleep(Duration::from_millis(50));
        }
    }

    /// Stop the vehicle for specified duration (in seconds)
    fn stop_vehicle(publisher: &Publisher<AckermannDriveStamped>, duration_sec: f64) {
        println!("Stop for {} seconds...", duration_sec);
        let iterations = (duration_sec * 20.0) as i32; // 50ms intervals = 20 per second

        for _ in 0..iterations {
            let msg = Self::create_drive_message(0.0, 0.0);
            let _ = publisher.publish(&msg);
            thread::sleep(Duration::from_millis(50));
        }
    }

    /// Create an AckermannDriveStamped message with specified speed and steering angle
    fn create_drive_message(speed: f32, steering_angle: f32) -> AckermannDriveStamped {
        let mut msg = AckermannDriveStamped::default();
        msg.drive.speed = speed;
        msg.drive.steering_angle = steering_angle;
        msg
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Rust Basic Drive");
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _basic_drive = F1tenthBasicDrive::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
