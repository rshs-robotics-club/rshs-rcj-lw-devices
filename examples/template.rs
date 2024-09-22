// you can use this code to access all the sensors.
// start your code at line 168.
// you shouldn't need to change anything before that

use bbr_irseeker::Irseeker;
use bno055::Bno055;
use raw::firmware::send_ledmode;
use rpi_build_hat_serial::*;
use rshs_rcj_lw_devices::bno055::BNO055OperationMode;
use rshs_rcj_lw_devices::button::Button;
use rshs_rcj_lw_devices::color::Color;
use rshs_rcj_lw_devices::omni::Omni;
use rshs_rcj_lw_devices::pollster::block_on;
use rshs_rcj_lw_devices::rppal::{hal::Delay, i2c::I2c};
use rshs_rcj_lw_devices::vl53l1x_uld::*;
use rshs_rcj_lw_devices::*;
use std::error::Error;
use std::time::Instant;
fn detect_bno055(i2c: &mut I2c) -> Result<u16, Box<dyn Error>> {
    let addresses = [0x28, 0x29];

    for &addr in &addresses {
        i2c.set_slave_address(addr)?;

        // Try reading the CHIP_ID register
        let mut buf = [0u8; 1];
        if i2c.block_read(0x00, &mut buf).is_ok() {
            if buf[0] == 0xA0 {
                return Ok(addr);
            }
        }
    }

    Err("BNO055 not detected on any known address".into())
}
fn main() {
    let _ = block_on(async move {
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
        const BNO055: u8 = 0b1000_0000;
        const ERR: &str = "Failed to communicate";

        let mut bno_addr: u8 = 0x29;

        let mut multiplexer = xca9548a::Xca9548a::new(i2c_mult, xca9548a::SlaveAddr::Default);
        // let mut colors = rshs_rcj_lw_devices::color::Color::new(i2c_color).unwrap();
        let mut lasers = vl53l1x_uld::VL53L1X::new(i2c_laser, vl53l1x_uld::DEFAULT_ADDRESS);
        let mut omni = Omni::new().await.unwrap();
        let mut colors = Color::new(i2c_color, 0x10).unwrap();
        let mut button = Button::new(i2c_button, 0x0c).unwrap();
        let mut irseeker = Irseeker::new().await.unwrap();

        let _ = multiplexer.select_channels(BNO055);
        match detect_bno055(&mut i2c_gyro) {
            Ok(addr) => bno_addr = addr as u8,
            Err(e) => println!("Error: {}", e),
        }
        let mut imu = if (bno_addr == 0x29) {
            Bno055::new(i2c_gyro)
        } else {
            Bno055::new(i2c_gyro).with_alternative_address()
        };

        let mut abs_facing = 0.0;
        let mut offset_angle = 0.0;

        let mut color_offset: [[i32; 4]; 4] = [[0, 0, 0, 0]; 4];
        let mut dist_left = 0;
        let mut dist_right = 0;
        let mut dist_back = 0;

        let mut color_left: [i32; 4];
        let mut color_front: [i32; 4];
        let mut color_right: [i32; 4];
        let mut color_back: [i32; 4];

        let mut ball_direction = 0;
        let mut ball_strength = 0;

        let mut start = false;
        let mut last_pressed = Instant::now();

        // set up gyro
        let _ = multiplexer.select_channels(BNO055);
        let _ = imu
            .init(&mut delay)
            .expect("An error occurred while building the IMU");
        let _ = imu
            .set_mode(BNO055OperationMode::NDOF, &mut delay)
            .expect("An error occurred while setting the IMU mode");
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

        loop {
            // actual loop
            // read irseeker
            ball_direction = irseeker.get_direction().await;
            ball_strength = irseeker.get_strength().await;
            let mut dummy = [0, 0, 0, 0];
            // read color sensors
            let _ = multiplexer.select_channels(COLOR_LEFT);
            dummy = colors.read_rgb().unwrap();
            color_left = [
                dummy[0] as i32,
                dummy[1] as i32,
                dummy[2] as i32,
                dummy[3] as i32,
            ];

            let _ = multiplexer.select_channels(COLOR_FRONT);
            dummy = colors.read_rgb().unwrap();
            color_front = [
                dummy[0] as i32,
                dummy[1] as i32,
                dummy[2] as i32,
                dummy[3] as i32,
            ];

            let _ = multiplexer.select_channels(COLOR_RIGHT);
            dummy = colors.read_rgb().unwrap();
            color_right = [
                dummy[0] as i32,
                dummy[1] as i32,
                dummy[2] as i32,
                dummy[3] as i32,
            ];

            let _ = multiplexer.select_channels(COLOR_BACK);
            dummy = colors.read_rgb().unwrap();
            color_back = [
                dummy[0] as i32,
                dummy[1] as i32,
                dummy[2] as i32,
                dummy[3] as i32,
            ];

            // read gyro
            let _ = multiplexer.select_channels(BNO055);
            match imu.euler_angles() {
                Ok(val) => {
                    abs_facing = val.c;
                    if (abs_facing >= 180.0) {
                        abs_facing = -(360.0 - abs_facing);
                    } // changes the angle value from 0-360 to -180-180
                }
                Err(e) => {
                    eprintln!("{:?}", e);
                }
            }
            let facing = abs_facing - offset_angle;

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

            if (button.is_pressed().unwrap() && !start && last_pressed.elapsed().as_millis() >= 200)
            {
                start = true;
                let _ = block_on(async move {
                    let mut serial = UART_SERIAL.lock().await;
                    println!("sending led mode");
                    let _ = send_ledmode(&mut serial, raw::firmware::LedMode::Green).await;
                });
                offset_angle = abs_facing;
                for i in 0..4 {
                    color_offset[0][i] = (color_left[i]) - 10000;
                }
                for i in 0..4 {
                    color_offset[1][i] = (color_front[i]) - 10000;
                }
                for i in 0..4 {
                    color_offset[2][i] = (color_right[i]) - 10000;
                }
                for i in 0..4 {
                    color_offset[3][i] = (color_back[i]) - 10000;
                }

                last_pressed = Instant::now();
            } else if (button.is_pressed().unwrap()
                && start
                && last_pressed.elapsed().as_millis() >= 200)
            {
                start = false;
                let _ = block_on(async move {
                    let mut serial = UART_SERIAL.lock().await;
                    println!("sending led mode");
                    let _ = send_ledmode(&mut serial, raw::firmware::LedMode::Orange).await;
                });
                last_pressed = Instant::now();
            }
            let facing = abs_facing - offset_angle;

            for i in 0..4 {
                color_left[i] -= color_offset[0][i];
            }
            for i in 0..4 {
                color_front[i] -= color_offset[1][i];
            }
            for i in 0..4 {
                color_right[i] -= color_offset[2][i];
            }
            for i in 0..4 {
                color_back[i] -= color_offset[3][i];
            }
            // ===============================================================================
            // ===========================||actual code here||================================
            // ===============================================================================
            // 1. dist_left, dist_right and dist_back are the distance that each laser sensor recieves
            // 2. color_left, color_front, color_right and color_back are the rgb and white values that each color sensor picks up.
            //    * the 4th value should be all you need to do line sensing
            // 3. facing is the angle that the robot is currently facing.
            // 4. pressing the button would cause the "start" variable to change into true or false.
            //    * you can use this variable to determine if the motors should move.
            //    * sensor values would also be calibrated when the button is pressed to make "start" true.
            //      * facing resets to 0 and color values reset to 10000, so make sure that the sensor does not start on the black line.


        }
    });
}