use std::error::Error;

use rppal::i2c::I2c;
const _ADDR: u16 = 0x10;
const _CONF: u8 = b'\x00';
const _REG_RED: u8 = 0x08;
const _REG_GREEN: u8 = 0x09;
const _REG_BLUE: u8 = 0x0A;
const _REG_WHITE: u8 = 0x0B;
const _DEFAULT_SETTINGS: u8 = b'\x00';  // initialise gain:1x, integration 40ms, Green Sensitivity 0.25168, Max. Detectable Lux 16496
                                        // No Trig, Auto mode, enabled.
const _SHUTDOWN: u8 = b'\x01';          // Disable colour sensor
const _INTEGRATION_TIME: u8 = 40;       // ms
const _G_SENSITIVITY: f32 = 0.25168;     // lux/step
pub struct Color{
    i2c: I2c,
}
impl Color {
    pub fn new(mut i2c: I2c, address: u8) -> Result<Self, Box<dyn Error>> {
        let _ = i2c.set_slave_address(address as u16);
        Ok(Self {i2c})
    }

    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        let _ = self.i2c.write(&[_CONF, _SHUTDOWN]);
        let _ = self.i2c.write(&[_CONF, _DEFAULT_SETTINGS]);
        Ok(())
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