use core::fmt;
use std::f32::consts::PI;
use std::thread;
use std::time::Duration;
use rppal::i2c;
use std::error::Error;
use std::fmt::{Debug, Display};
use nalgebra::{Vector2, Vector3};
use libm::{powf, atan2f, sqrtf};
use crate::bits;
use crate::device::{AccelRange, GyroRange, ACCEL_CONFIG, ACCEL_HPF, ACCEL_SENS, ACC_REGX_H, CLKSEL, DEFAULT_SLAVE_ADDR, GYRO_CONFIG, GYRO_REGX_H, GYRO_SENS, INT_ENABLE, INT_PIN_CFG, INT_STATUS, MOT_DUR, MOT_THR, PWR_MGMT_1, TEMP_OFFSET, TEMP_OUT_H, TEMP_SENSITIVITY, WHOAMI};

pub const PI_180: f32 = PI / 180.0;

#[derive(Debug)]
pub enum Mpu6050Error {
    I2cError(i2c::Error),
    InvalidChipID(u8),
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
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}
impl Mpu6050 {
    pub fn new() -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(0x68)?;
        Ok(Self {
            i2c: i,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        })
    }
    pub fn new_with_addr(addr: u8) -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(addr.into())?;
        Ok(Self {
            i2c: i,
            acc_sensitivity: ACCEL_SENS.0,
            gyro_sensitivity: GYRO_SENS.0,
        })
    }

    pub fn new_with_sens(arange: AccelRange, grange: GyroRange) -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(0x68)?;
        Ok(Self {
            i2c: i,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        })
    }

    pub fn new_with_addr_and_sens(addr: u8, arange: AccelRange, grange: GyroRange) -> Result<Self, Mpu6050Error> {
        let mut i = i2c::I2c::new()?;
        i.set_slave_address(addr.into())?;
        Ok(Self {
            i2c: i,
            acc_sensitivity: arange.sensitivity(),
            gyro_sensitivity: grange.sensitivity(),
        })
    }

    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Mpu6050Error> {
        self.i2c
            .write(&[reg, byte])
            .map_err(Mpu6050Error::I2cError)?;
        Ok(())
    }
    pub fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), Mpu6050Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0])?)
    }
    pub fn write_bits(&mut self, reg: u8, start_bit: u8, length: u8, data: u8) -> Result<(), Mpu6050Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        Ok(self.write_byte(reg, byte[0])?)
    }

    pub fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Mpu6050Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    pub fn read_bits(&mut self, reg: u8, start_bit: u8, length: u8) -> Result<u8, Mpu6050Error> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bits(byte[0], start_bit, length))
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

    fn wake(&mut self) -> Result<(), Mpu6050Error> {
        let _ = self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        thread::sleep(Duration::from_millis(100u8.into()));
        Ok(())
    }

    pub fn get_clock_source(&mut self) -> Result<CLKSEL, Mpu6050Error> {
        let source = self.read_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL.bit, PWR_MGMT_1::CLKSEL.length)?;
        Ok(CLKSEL::from(source))
    }

    pub fn init(&mut self) -> Result<(), Mpu6050Error> {
        self.wake()?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        self.set_accel_hpf(ACCEL_HPF::_RESET)?;
        Ok(())
    }

    fn verify(&mut self) -> Result<(), Mpu6050Error> {
        let address = self.read_byte(WHOAMI)?;
        if address != DEFAULT_SLAVE_ADDR {
            return Err(Mpu6050Error::InvalidChipID(address));
        }
        Ok(())
    }
    pub fn setup_motion_detection(&mut self) -> Result<(), Mpu6050Error> {
        self.write_byte(0x6B, 0x00)?;
        // optional? self.write_byte(0x68, 0x07)?; // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20)?; //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01)?; //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
        self.write_byte(MOT_THR, 10)?; //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
        self.write_byte(MOT_DUR, 40)?; //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
        self.write_byte(0x69, 0x15)?; //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
        self.write_byte(INT_ENABLE::ADDR, 0x40)?; //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
        Ok(())
    }

    pub fn get_motion_detect(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(INT_STATUS::ADDR, INT_STATUS::MOT_INT)? != 0)
    }

    pub fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Mpu6050Error> {
        Ok(
            self.write_bits(ACCEL_CONFIG::ADDR,
            ACCEL_CONFIG::ACCEL_HPF.bit,
            ACCEL_CONFIG::ACCEL_HPF.length,
            mode as u8)?
        )
    }

    pub fn get_accel_hpf(&mut self) -> Result<ACCEL_HPF, Mpu6050Error> {
        let mode: u8 = self.read_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ACCEL_HPF.bit, ACCEL_CONFIG::ACCEL_HPF.length)?;
        Ok(ACCEL_HPF::from(mode))
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error> {
        self.write_bits(GYRO_CONFIG::ADDR, GYRO_CONFIG::FS_SEL.bit, GYRO_CONFIG::FS_SEL.length, range as u8)?;
        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    pub fn get_gyro_range(&mut self) -> Result<GyroRange, Mpu6050Error> {
        let byte = self.read_bits(GYRO_CONFIG::ADDR, GYRO_CONFIG::FS_SEL.bit, GYRO_CONFIG::FS_SEL.length)?;
        Ok(GyroRange::from(byte))
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error> {
        self.write_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::FS_SEL.bit, ACCEL_CONFIG::FS_SEL.length, range as u8)?;
        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }
    pub fn get_accel_range(&mut self) -> Result<AccelRange, Mpu6050Error> {
        let byte = self.read_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::FS_SEL.bit, ACCEL_CONFIG::FS_SEL.length)?;
        Ok(AccelRange::from(byte))
    }
    pub fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)?)
    }
    pub fn get_sleep_enabled(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP)? != 0)
    }

    pub fn set_temp_enabled(&mut self, enable: bool) -> Result<(), Mpu6050Error> {
        Ok(self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)?)
    }
    pub fn get_temp_enabled(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)? == 0)
    }

    /// set accel x self test
    pub fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)?)
    }

    /// get accel x self test
    pub fn get_accel_x_self_test(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)? != 0)
    }

    /// set accel y self test
    pub fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)?)
    }

    /// get accel y self test
    pub fn get_accel_y_self_test(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)? != 0)
    }

    /// set accel z self test
    pub fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), Mpu6050Error> {
        Ok(self.write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)?)
    }

    /// get accel z self test
    pub fn get_accel_z_self_test(&mut self) -> Result<bool, Mpu6050Error> {
        Ok(self.read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)? != 0)
    }
    
    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on MPU6050
    /// https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
    pub fn get_acc_angles(&mut self) -> Result<Vector2<f32>, Mpu6050Error> {
        let acc = self.get_acc()?;

        Ok(Vector2::<f32>::new(
            atan2f(acc.y, sqrtf(powf(acc.x, 2.) + powf(acc.z, 2.))),
            atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.)))
        ))
    }

    /// Converts 2 bytes number in 2 compliment
    /// TODO i16?! whats 0x8000?!
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }

    /// Reads rotation (gyro/acc) from specified register
    fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Mpu6050Error> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32
        ))
    }

    /// Accelerometer readings in g
    pub fn get_acc(&mut self) -> Result<Vector3<f32>, Mpu6050Error> {
        let mut acc = self.read_rot(ACC_REGX_H)?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub fn get_gyro(&mut self) -> Result<Vector3<f32>, Mpu6050Error> {
        let mut gyro = self.read_rot(GYRO_REGX_H)?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub fn get_temp(&mut self) -> Result<f32, Mpu6050Error> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf)?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        // According to revision 4.2
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }
}
