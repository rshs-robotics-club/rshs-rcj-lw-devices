use gpio::{Gpio, InputPin};
use i2c::I2c;
// this is a wrapper for the lsm303dlhc magnetometer.
use rppal::*;
use std::error::Error;
use lsm303dlhc::{self, Lsm303dlhc};

struct Compass{
    lsm303dlhc: Lsm303dlhc<I2c>,
    drdy_pin: InputPin,
}
impl Compass {
    pub fn new(i2c: I2c, ready_pin: u8) -> Result<Self, Box<dyn Error>> {
        let lsm = Lsm303dlhc::new(i2c).unwrap();
        let pin = Gpio::new().unwrap().get(ready_pin).unwrap().into_input_pulldown();
        Ok(Self {lsm303dlhc: lsm, drdy_pin: pin})
    }
    pub fn ready(&mut self) -> Result<bool, Box<dyn Error>> {
        Ok(self.drdy_pin.is_high())
    }
    pub fn get_yaw(&mut self) -> Result<f32, Box<dyn Error>> {
        let v = self.lsm303dlhc.mag().unwrap();
        Ok(f32::atan2(v.z as f32, v.x as f32).to_degrees())
    }
    pub fn get_yaw_radian(&mut self) -> Result<f32, Box<dyn Error>> {
        let v = self.lsm303dlhc.mag().unwrap();
        Ok(f32::atan2(v.z as f32, v.x as f32))
    }
}