use rppal::i2c::I2c;
use std::error::Error;
pub struct Button {
    i2c: I2c,
}

impl Button {
    pub fn new(mut i2c: I2c, address: u8) -> Result<Self, Box<dyn Error>> {
        let _ = i2c.set_slave_address(address as u16);
        Ok(Self {i2c})
    }

    pub fn is_pressed(&mut self) -> Result<bool, Box<dyn Error>> {
        let reg = 0x11;  // Example register for the button state
        let mut buf = [0u8];
        let _ = self.i2c.block_read( reg, &mut buf)?;
        Ok(buf[0] & 0x01 != 1)
    }

    pub fn was_pressed(&mut self) -> Result<bool, Box<dyn Error>> {
        let reg = 0x12;  // Example register for was_pressed state
        let mut buf = [0u8];
        let _ = self.i2c.block_read(reg, &mut buf)?;
        Ok(buf[0] & 0x01 != 1)
    }

    pub fn double_clicked(&mut self) -> Result<bool, Box<dyn Error>> {
        let reg = 0x13;  // Example register for DOUBLE_PRESS_DETECTED state
        let mut buf = [0u8];
        let _ = self.i2c.block_read(reg, &mut buf)?;
        Ok(buf[0] & 0x01 != 1)
    }

    pub fn double_click_duration(&mut self, time: u16) -> Result<(), Box<dyn Error>> {
        let reg = 0x21;  // Example register for DOUBLE_PRESS_DETECTED state
        let _  =self.i2c.block_write(reg, &u16::to_be_bytes(time));
        Ok(())
    }

    /// amount of clicks after last time the sensor was used
    pub fn press_count(&mut self) -> Result<u16, Box<dyn Error>> {
        let reg = 0x14;  // Example register for press count
        let mut buf = [0;2];
        let _ = self.i2c.block_read(reg, &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }
}