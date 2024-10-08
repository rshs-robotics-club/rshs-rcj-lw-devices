use std::error::Error;

use rppal::i2c::I2c;
pub struct Irseeker {
    ir: I2c,
}
impl Irseeker {
    /// creates a new Irseeker object.
    pub async fn new() -> Result<Self, Box<dyn Error>> {
        let mut ir_i2c = I2c::new()?;
        let _ = ir_i2c.set_slave_address(0x8);
        Ok(Self { ir: ir_i2c })
    }
    pub async fn get_direction(&mut self) -> u8 {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(&mut buf);
        buf[0]
    }
    pub async fn get_strength(&mut self) -> u8 {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(&mut buf);
        buf[1]
    }
    pub async fn get_both(&mut self) -> [u8; 2] {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(&mut buf);
        buf
    }
}
