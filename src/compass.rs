use gpio::{Gpio, InputPin};
use i2c::I2c;
// this is a wrapper for the lsm303dlhc magnetometer.
use lsm303dlhc::{self, Lsm303dlhc};
use rppal::*;
use std::{
    error::Error,
    time::Instant,
};

/// instructions:
///
/// call the funcitons in this order to set up the sensor.
///
/// ```new()```
///
/// ```get_offsets()```
///
/// ```set_zero()```
///
/// after that, use ```ready()``` to check if the sensor is ready to output values
/// and use ```get_raw_rel()``` to get the angle of the sensor
pub struct Compass {
    pub lsm303dlhc: Lsm303dlhc<I2c>,
    pub drdy_pin: InputPin,
    pub start_angle: f32,
    pub x_offset: f32,
    pub x_scale: f32,
    pub z_offset: f32,
    pub z_scale: f32,
}

impl Compass {
    pub async fn new(i2c: I2c, ready_pin: u8) -> Result<Self, Box<dyn Error>> {
        let lsm = Lsm303dlhc::new(i2c).unwrap();
        let pin = Gpio::new()
            .unwrap()
            .get(ready_pin)
            .unwrap()
            .into_input_pulldown();
        Ok(Self {
            lsm303dlhc: lsm,
            drdy_pin: pin,
            start_angle: 0.0,
            x_offset: 0.0,
            x_scale: 0.0,
            z_offset: 0.0,
            z_scale: 0.0,
        })
    }

    /// checks if the sensor is ready using the Data-Ready (DRDY) pin
    pub async fn ready(&mut self) -> Result<bool, Box<dyn Error>> {
        Ok(self.drdy_pin.is_high())
    }

    /// gets the raw yaw value (not recommended)
    pub async unsafe fn get_yaw_raw(&mut self) -> Result<f32, Box<dyn Error>> {
        let v = self.lsm303dlhc.mag().unwrap();
        Ok(f32::atan2(v.z as f32, v.x as f32).to_degrees())
    }

    /// set the starting angle to be the angle facing right now
    pub async fn set_zero(&mut self) -> Result<(), Box<dyn Error>> {
        self.start_angle = self.get_yaw_abs().await.unwrap();
        Ok(())
    }

    /// gets the true bearing of the sensor
    pub async fn get_yaw_abs(&mut self) -> Result<f32, Box<dyn Error>> {
        let values = self.lsm303dlhc.mag().unwrap();
        let calibrated_z = (values.z as f32 - self.z_offset) / self.z_scale;
        let calibrated_x = (values.x as f32 - self.x_offset) / self.x_scale;
        let abs_angle = f32::atan2(calibrated_z, calibrated_x).to_degrees();
        Ok(abs_angle)
    }

    /// gets the angle relative to the start_angle
    ///
    /// you would need to use the ```set_zero()``` function before this.
    pub async fn get_yaw_rel(&mut self) -> Result<f32, Box<dyn Error>> {
        let mut ans = self.get_yaw_abs().await.unwrap() - self.start_angle;
        while ans < -180.0 {
            ans += 180.0;
        }
        while ans > 180.0 {
            ans -= 180.0;
        }
        Ok(ans)
    }

    /// gets the offset and scale for the x and z axis.
    ///
    /// spin the sensor slowly around for at least 1 loop.
    ///
    /// as long as the sensor can record both the highest and the lowest value, it should work.
    pub async fn get_offsets(&mut self, seconds: f32) -> Result<(), Box<dyn Error>> {
        println!("start rotating the sensor.");
        println!("make sure that you rotate it at least 1 time");
        println!("do not tilt it to other directions");
        let mut x_max = 0.0;
        let mut x_min = 0.0;
        let mut z_max = 0.0;
        let mut z_min = 0.0;
        let start_time = Instant::now();
        while start_time.elapsed().as_secs_f32() <= seconds {
            let values = self.lsm303dlhc.mag().unwrap();
            if (values.x as f32) > x_max {
                x_max = values.x as f32
            };
            if (values.x as f32) < x_min {
                x_min = values.x as f32
            };
            if (values.z as f32) > z_max {
                z_max = values.z as f32
            };
            if (values.z as f32) < z_min {
                z_min = values.z as f32
            };
        }
        self.x_offset = (x_max + x_min) / 2.0;
        self.x_scale = (x_max - x_min) / 2.0;
        self.z_offset = (z_max + z_min) / 2.0;
        self.z_scale = (z_max - z_min) / 2.0;
        Ok(())
    }
}
