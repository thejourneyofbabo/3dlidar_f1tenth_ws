use ackermann_msgs::msg::AckermannDriveStamped;
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::f32::consts::PI;

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
                if let Err(e) = Self::process_and_publish_filtered(msg, &publisher_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        Ok(Self {
            scan_subscription,
            drive_publisher,
        })
    }

    fn process_and_publish_filtered(
        msg: LaserScan,
        publisher: &Publisher<LaserScan>,
    ) -> Result<(), Error> {
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
