// ================================
// src/main.rs
// ================================
use anyhow::Result;
use module_gap::ReactiveFollowGap;
use rclrs::*; // f1tenth_gap_follow → module_gap으로 변경

fn main() -> Result<(), RclrsError> {
    println!("F1Tenth Gap Follow Node with Rust (Modularized)");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _node = ReactiveFollowGap::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}
