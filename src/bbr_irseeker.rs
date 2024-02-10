use rppal::i2c::{self, *};


pub struct Irseeker{
    ir: I2c,
}
impl Irseeker{
    /// creates a new Irseeker object.
    pub async unsafe fn new() -> Result<Self>{
        let mut ir_i2c = i2c::I2c::new()?;
        ir_i2c.set_slave_address(0x8);
        Ok(Self {ir: ir_i2c})
    }
    pub async unsafe fn get_direction(&mut self) -> u8{
        let mut buf: [u8; 2] = [0,0];
        self.ir.read(&mut buf);
        buf[0]
    }
    pub async unsafe fn get_strength(&mut self) -> u8{
        let mut buf: [u8; 2] = [0,0];
        self.ir.read(&mut buf);
        buf[1]
    }
    pub async unsafe fn get_both(&mut self) -> [u8; 2]{
        let mut buf: [u8; 2] = [0,0];
        self.ir.read(&mut buf);
        buf
    }
}