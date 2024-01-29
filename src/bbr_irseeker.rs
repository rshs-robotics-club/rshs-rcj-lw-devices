use rppal::i2c::{self, *};


struct Irseeker{
    wire: I2c,
}
impl Irseeker{
    /// creates a new Irseeker object.
    async unsafe fn new() -> Self{
        let mut ir_i2c = i2c::I2c::new().unwrap();
        ir_i2c.set_slave_address(0x8);
        Self {wire: ir_i2c}
    }
    async unsafe fn new_custom_address(addr: u8) -> Self{
        let mut ir_i2c = i2c::I2c::new().unwrap();
        ir_i2c.set_slave_address(addr.into());
        Self {wire: ir_i2c}
    }
    async unsafe fn get_direction(&mut self) -> u8{
        let mut buf: [u8; 1] = [0; 1];
        let _ = self.wire.read(&mut buf);
        buf[0]
    }
}