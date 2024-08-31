use std::error::Error;

use rppal::i2c::I2c;
const _ADDR: u16 = 0x10;
const _CONF: u8 = b'\x00';
const _REG_RED: u8 = 0x08;
const _REG_GREEN: u8 = 0x09;
const _REG_BLUE: u8 = 0x0A;
const _REG_WHITE: u8 = 0x0B;

pub struct Color{
    i2c: I2c,
}
impl Color {
    pub fn new(address: u16) -> Result<Self, Box<dyn Error>> {
        let mut bus = I2c::new()?;
        let _ = bus.set_slave_address(address);
        Ok(Self {i2c: bus})
    }
    pub fn read_rgb(&mut self) -> Result<[u16; 4], Box<dyn Error>>{
        let mut value: [u8; 2] = [0, 0];
        let _ = self.i2c.write_read(&[_REG_RED], &mut value);
        let u16red = u16::from_le_bytes(value);
        let _ = self.i2c.write_read(&[_REG_GREEN], &mut value);
        let u16green = u16::from_le_bytes(value);
        let _ = self.i2c.write_read(&[_REG_BLUE], &mut value);
        let u16blue = u16::from_le_bytes(value);
        let _ = self.i2c.write_read(&[_REG_WHITE], &mut value);
        let data_white_int = u16::from_le_bytes(value);
        Ok([u16red, u16green, u16blue, data_white_int])
    }
}