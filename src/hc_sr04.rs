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
    pub fn new_with_addr(addr: u8) -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(addr.into())?;
        Ok(Self { i2c: i })
    }

    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Mpu6050Error> {
        self.i2c
            .write(&[reg, byte])
            .map_err(Mpu6050Error::I2cError)?;
        Ok(())
    }
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(&[reg], &mut byte)
            .map_err(Mpu6050Error::I2cError)?;
        Ok(byte[0])
    }
    pub fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Mpu6050Error> {
        self.i2c
            .write_read(&[reg], buf)
            .map_err(Mpu6050Error::I2cError)?;
        Ok(())
    }
}
