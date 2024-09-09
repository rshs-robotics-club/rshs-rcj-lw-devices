use rppal::i2c::I2c;
use std::error::Error;
const _BASE_ADDRESS: u8 = 0x35;
const _DEVICE_ID: u16 = 578;
const _REG_STATUS: u8 = 0x08;
const _REG_FIRM_MAJ: u8 = 0x02;
const _REG_FIRM_MIN: u8 = 0x03;
const _REG_I2C_ADDRESS: u8 = 0x04;
const _REG_RAW: u8 = 0x05;
const _REG_PERIOD: u8 = 0x06;
const _REG_LED: u8 = 0x07;
const _REG_SELF_TEST: u8 = 0x09;
const _REG_WHOAMI: u8 = 0x01;
fn _set_bit(x: u8, n: u8) -> u8 {
    x | (1 << n)
}
pub struct Ultrasonic {
    i2c: I2c,
    address: u16,
    millimeters_per_microsecond: f32,
}
impl Ultrasonic {
    pub fn new(
        i2c: I2c,
        address: u16,
        speed_of_sound: f32,
    ) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            i2c: i2c,
            address: address,
            millimeters_per_microsecond: speed_of_sound,
        })
    }
    pub fn check_error(&mut self) -> Result<(), Box<dyn Error>> {
        if self.whoami() != _DEVICE_ID {
            let message = format!("Incorrect device found at {}", self.address);
            panic!("{}", message);
        }
        Ok(())
    }
    pub fn _read_int(&mut self, register: u8) -> u16 {
        let mut data: [u8; 2] = [0; 2];
        let _ = self.i2c.write_read(&[register], &mut data);
        u16::from_be_bytes(data)
    }
    pub fn _write_int(&mut self, register: u8, integer: u8) {
        let _ = self.i2c.write(&[register, integer.to_be_bytes()[0]]);
    }
    pub fn distance_mm(&mut self) -> Result<f32, Box<dyn Error>> {
        let trip_time = self._read_int(_REG_RAW) as f32;
        Ok(trip_time * self.millimeters_per_microsecond / 2.0)
    }
    pub fn new_sample_available(&mut self) -> Result<bool, Box<dyn Error>> {
        let status = self._read_int(_REG_STATUS);
        Ok((status & 0x01) != 0)
    }
    pub fn address(&mut self) -> u16 {
        self.address
    }
    pub fn check_led_state(&mut self) -> Result<bool, Box<dyn Error>> {
        Ok(self._read_int(_REG_LED) != 0)
    }
    pub fn set_led(&mut self, on: bool) -> Result<(), Box<dyn Error>> {
        self._write_int(_set_bit(_REG_LED, 7), on as u8);
        Ok(())
    }
    pub fn whoami(&mut self) -> u16 {
        self._read_int(_REG_WHOAMI)
    }
}
