use std::error::Error;

use embedded_hal::i2c::I2c;
pub struct Irseeker {
    ir: I2c,
    addr: u8,
}
impl Irseeker {
    /// creates a new Irseeker object.
    pub async fn new(mut ir_i2c: I2c, address: u8) -> Result<Self, Box<dyn Error>> {
        Ok(Self { ir: ir_i2c, addr: address })
    }
    pub async fn get_direction(&mut self) -> u8 {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(self.addr, &mut buf);
        buf[0]
    }
    pub async fn get_strength(&mut self) -> u8 {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(self.addr, &mut buf);
        buf[1]
    }
    pub async fn get_both(&mut self) -> [u8; 2] {
        let mut buf: [u8; 2] = [0, 0];
        let _ = self.ir.read(self.addr, &mut buf);
        buf
    }
}
