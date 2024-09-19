// you can use this code to access all the sensors.
// start your code at line 137.
// you shouldn't need to change anything before that

use std::time::Instant;

use bbr_irseeker::Irseeker;
use rshs_rcj_lw_devices::button::Button;
use rshs_rcj_lw_devices::bno055::BNO055OperationMode;
use rshs_rcj_lw_devices::omni::Omni;
use rshs_rcj_lw_devices::rppal::{hal::Delay, i2c::I2c};
use rshs_rcj_lw_devices::*;
use rshs_rcj_lw_devices::color::Color;
use rshs_rcj_lw_devices::vl53l1x_uld::*;
use rshs_rcj_lw_devices::pollster::block_on;
fn main() {let _ = block_on(async move {
    let mut i2c_color = I2c::new().unwrap();
    let mut i2c_gyro = I2c::new().unwrap();
    let mut i2c_mult = I2c::new().unwrap();
    let mut i2c_laser = I2c::new().unwrap();
    let mut i2c_button = I2c::new().unwrap();
    let mut delay = Delay {};
    const COLOR_LEFT: u8 = 0b0100_0000;
    const COLOR_FRONT: u8 = 0b0000_0001;
    const COLOR_RIGHT: u8 = 0b0000_0100;
    const COLOR_BACK: u8 = 0b0001_0000;
    const LASER_LEFT: u8 = 0b0010_0000;
    const LASER_RIGHT: u8 = 0b0000_0010;
    const LASER_BACK: u8 = 0b0000_1000;

    const ERR : &str = "Failed to communicate";

    let mut multiplexer = xca9548a::Xca9548a::new(i2c_mult, xca9548a::SlaveAddr::Default);
    // let mut colors = rshs_rcj_lw_devices::color::Color::new(i2c_color).unwrap();
    let mut lasers = vl53l1x_uld::VL53L1X::new(i2c_laser, vl53l1x_uld::DEFAULT_ADDRESS);
    let mut imu = bno055::Bno055::new(i2c_gyro).with_alternative_address();
    let mut omni = Omni::new().await.unwrap();
    let mut colors = Color::new(i2c_color, 0x10).unwrap();
    let mut button = Button::new(i2c_button, 0x0c).unwrap();
    let mut irseeker = Irseeker::new().await.unwrap();
    let mut facing = 0.0;

    let mut dist_left = 0;
    let mut dist_right = 0;
    let mut dist_back = 0;

    let mut color_left: [u16; 4];
    let mut color_front: [u16; 4];
    let mut color_right: [u16; 4];
    let mut color_back: [u16; 4];

    let mut ball_direction = 0;
    let mut ball_strength = 0;

    let mut start = false;
    let mut last_pressed = Instant::now();

    // set up gyro
    let _ = imu.init(&mut delay).expect("An error occurred while building the IMU");
    let _ = imu.set_mode(BNO055OperationMode::NDOF, &mut delay).expect("An error occurred while setting the IMU mode");
    let calib = imu.calibration_profile(&mut delay).unwrap();
    let _ = imu.set_calibration_profile(calib, &mut delay).unwrap();
    
    // set up laser sensors
    let _ = multiplexer.select_channels(LASER_LEFT);
    let _ = lasers.init(IOVoltage::Volt2_8).expect(ERR);
    let _ = lasers.set_distance_mode(DistanceMode::Long);
    let _ = lasers.start_ranging();

    let _ = multiplexer.select_channels(LASER_RIGHT);
    let _ = lasers.init(IOVoltage::Volt2_8).expect(ERR);
    let _ = lasers.set_distance_mode(DistanceMode::Long);
    let _ = lasers.start_ranging();

    let _ = multiplexer.select_channels(LASER_BACK);
    let _ = lasers.init(IOVoltage::Volt2_8).expect(ERR);
    let _ = lasers.set_distance_mode(DistanceMode::Long);
    let _ = lasers.start_ranging();

    // set up color sensors
    let _ = multiplexer.select_channels(COLOR_LEFT);
    let _ = colors.init();

    let _ = multiplexer.select_channels(COLOR_FRONT);
    let _ = colors.init();

    let _ = multiplexer.select_channels(COLOR_RIGHT);
    let _ = colors.init();

    let _ = multiplexer.select_channels(COLOR_BACK);
    let _ = colors.init();

    loop { // actual loop
        // read irseeker
        ball_direction = irseeker.get_direction().await;
        ball_strength = irseeker.get_strength().await;

        // read color sensors
        let _ = multiplexer.select_channels(COLOR_LEFT);
        color_left = colors.read_rgb().unwrap();

        let _ = multiplexer.select_channels(COLOR_FRONT);
        color_front = colors.read_rgb().unwrap();

        let _ = multiplexer.select_channels(COLOR_RIGHT);
        color_right = colors.read_rgb().unwrap();

        let _ = multiplexer.select_channels(COLOR_BACK);
        color_back = colors.read_rgb().unwrap();

        // read gyro
        match imu.euler_angles() {
            Ok(val) => {
                facing = val.c;
                if (facing >= 180.0) {facing = -(360.0-facing);} // changes the angle value from 0-360 to -180-180
            }
            Err(e) => {
                eprintln!("{:?}", e);
            }
        }

        // read laser sensors
        let _ = multiplexer.select_channels(LASER_LEFT);
        // Retrieve measured distance.
        dist_left = lasers.get_distance().expect(ERR);

        let _ = multiplexer.select_channels(LASER_RIGHT);
        // Retrieve measured distance.
        dist_right = lasers.get_distance().expect(ERR);

        let _ = multiplexer.select_channels(LASER_BACK);
        // Retrieve measured distance.
        dist_back = lasers.get_distance().expect(ERR);

        if (button.is_pressed().unwrap() && !start && last_pressed.elapsed().as_millis() >= 200) {
            start = true;
            last_pressed = Instant::now();
        }
        else if (button.is_pressed().unwrap() && start && last_pressed.elapsed().as_millis() >= 200) {
            start = false;
            last_pressed = Instant::now();
        }
        // ===============================================================================
        // ===========================||actual code here||================================
        // ===============================================================================
        // 1. dist_left, dist_right and dist_back are the distance that each laser sensor recieves
        // 2. color_left, color_front, color_right and color_back are the rgb and white values that each color sensor picks up.
        //      the 4th value should be all you need to do line sensing
        // 3. facing is the angle that the robot faces.
        
        

    }
});}
