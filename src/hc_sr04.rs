use core::fmt;
use rppal::i2c;
use std::error::Error;
use std::fmt::{Debug, Display};
#[derive(Debug)]
pub enum Mpu6050Error {
    I2cError(i2c::Error),
}
impl Display for Mpu6050Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Mpu6050 Error is here!")
    }
}
impl Error for Mpu6050Error {}

impl From<rppal::i2c::Error> for Mpu6050Error {
    fn from(error: rppal::i2c::Error) -> Self {
        Mpu6050Error::I2cError(error)
    }
}

pub struct Mpu6050 {
    i2c: rppal::i2c::I2c,
}
impl Mpu6050 {
    pub fn new() -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(0x68)?;
        Ok(Self { i2c: i })
    }
}
