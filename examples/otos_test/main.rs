use std::{thread::sleep, time::Duration};

use otos::{Pose2d, OTOS};
use rshs_rcj_lw_devices::*;
fn main() {
    let i2c = rppal::i2c::I2c::new().unwrap();
    let mut otos = OTOS::new(0x17, i2c);

    let _ = otos.begin();
    println!("calibrating in 5 seconds");
    sleep(Duration::from_secs(5));
    otos.calibrate_imu(255, true);
    println!("finished calibration");
    otos.reset_tracking();
    otos.set_offset(Pose2d { x: -1.5, y: 1.5, h: 0.0 });
    // otos.set_offset(Pose2d {x: -40.0, y: 35.0, h: 0.0});
    loop {
        let xyh = otos.get_position().unwrap();
        println!("{}, {}, {}", xyh.x * (1.0 / 39.37) * 1000.0, xyh.y * (1.0 / 39.37) * 1000.0, xyh.h);
    }
    println!("Hello, world!");
}
