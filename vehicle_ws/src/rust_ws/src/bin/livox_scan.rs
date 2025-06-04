use anyhow::{Error, Result};
use rclrs::{self, Context};
use sensor_msgs::msg::PointCloud2;
use std::env;

fn main() -> Result<(), Error> {
    println!("This is LiDAR Scan node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "lidar_scanner")?;
    let _subscriber = node.create_subscription::<PointCloud2, _>(
        "/livox/lidar",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PointCloud2| {
            println!("{:?}", msg);
        },
    )?;

    rclrs::spin(node).map_err(|err| err.into())
}
