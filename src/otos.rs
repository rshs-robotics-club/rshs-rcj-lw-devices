use std::{error::Error, thread::sleep, time::Duration};

use rppal::i2c::I2c;

pub struct Pose2d {
    pub x: f32,
    pub y: f32,
    pub h: f32,
}
impl Pose2d {
    pub fn new(x: f32, y: f32, h: f32) -> Self {
        Self {x, y, h}
    }
}
/*
Class for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).
Includes methods to communicate with the sensor, such as getting the tracked
location, configuring the sensor, etc. This class is a base class that must
be derived to implement the delay function and I2C communication bus.
*/
// Set default name and I2C address(es)
const _DEFAULT_NAME: &str = "Qwiic OTOS";
const DEVICE_NAME: &str         = _DEFAULT_NAME;

// OTOS register map
const kRegProductId: u8 = 0x00;
const kRegHwVersion: u8 = 0x01;
const kRegFwVersion: u8 = 0x02;
const kRegScalarLinear: u8 = 0x04;
const kRegScalarAngular: u8 = 0x05;
const kRegImuCalib: u8 = 0x06;
const kRegReset: u8 = 0x07;
const kRegSignalProcess: u8 = 0x0E;
const kRegSelfTest: u8 = 0x0F;
const kRegOffXL: u8 = 0x10;
const kRegOffXH: u8 = 0x11;
const kRegOffYL: u8 = 0x12;
const kRegOffYH: u8 = 0x13;
const kRegOffHL: u8 = 0x14;
const kRegOffHH: u8 = 0x15;
const kRegStatus: u8 = 0x1F;
const kRegPosXL: u8 = 0x20;
const kRegPosXH: u8 = 0x21;
const kRegPosYL: u8 = 0x22;
const kRegPosYH: u8 = 0x23;
const kRegPosHL: u8 = 0x24;
const kRegPosHH: u8 = 0x25;
const kRegVelXL: u8 = 0x26;
const kRegVelXH: u8 = 0x27;
const kRegVelYL: u8 = 0x28;
const kRegVelYH: u8 = 0x29;
const kRegVelHL: u8 = 0x2A;
const kRegVelHH: u8 = 0x2B;
const kRegAccXL: u8 = 0x2C;
const kRegAccXH: u8 = 0x2D;
const kRegAccYL: u8 = 0x2E;
const kRegAccHL: u8 = 0x30;
const kRegAccHH: u8 = 0x31;
const kRegPosStdXL: u8 = 0x32;
const kRegPosStdXH: u8 = 0x33;
const kRegPosStdYL: u8 = 0x34;
const kRegPosStdYH: u8 = 0x35;
const kRegPosStdHL: u8 = 0x36;
const kRegPosStdHH: u8 = 0x37;
const kRegVelStdXL: u8 = 0x38;
const kRegVelStdXH: u8 = 0x39;
const kRegVelStdYL: u8 = 0x3A;
const kRegVelStdYH: u8 = 0x3B;
const kRegVelStdHL: u8 = 0x3C;
const kRegVelStdHH: u8 = 0x3D;
const kRegAccStdXL: u8 = 0x3E;
const kRegAccStdXH: u8 = 0x3F;
const kRegAccStdYL: u8 = 0x40;
const kRegAccStdYH: u8 = 0x41;
const kRegAccStdHL: u8 = 0x42;
const kRegAccStdHH: u8 = 0x43;

// Product ID register value
const kProductId: u8 = 0x5F;

// Conversion factors
const kMeterToInch: f32 = 39.37;
const kInchToMeter: f32 = 1.0 / kMeterToInch;
const kRadianToDegree: f32 = 180.0 / 3.14159;
const kDegreeToRadian: f32 = 3.14159 / 180.0;

// Conversion factor for the linear position registers. 16-bit signed
// registers with a max value of 10 meters (394 inches) gives a resolution
// of about 0.0003 mps (0.012 ips)
const kMeterToInt16: f32 = 32768.0 / 10.0;
const kInt16ToMeter: f32 = 1.0 / kMeterToInt16;

// Conversion factor for the linear velocity registers. 16-bit signed
// registers with a max value of 5 mps (197 ips) gives a resolution of about
// 0.00015 mps (0.006 ips)
const kMpsToInt16: f32 = 32768.0 / 5.0;
const kInt16ToMps: f32 = 1.0 / kMpsToInt16;

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
const kMpssToInt16: f32 = 32768.0 / (16.0 * 9.80665);
const kInt16ToMpss: f32 = 1.0 / kMpssToInt16;

// Conversion factor for the angular position registers. 16-bit signed
// registers with a max value of pi radians (180 degrees) gives a resolution
// of about 0.00096 radians (0.0055 degrees)
const kRadToInt16: f32 = 32768.0 / 3.14159;
const kInt16ToRad: f32 = 1.0 / kRadToInt16;

// Conversion factor for the angular velocity registers. 16-bit signed
// registers with a max value of 34.9 rps (2000 dps) gives a resolution of
// about 0.0011 rps (0.061 degrees per second)
const kRpsToInt16: f32 = 32768.0 / (2000.0 * kDegreeToRadian);
const kInt16ToRps: f32 = 1.0 / kRpsToInt16;

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a
// resolution of about 0.096 rps^2 (5.5 dps^2)
const kRpssToInt16: f32 = 32768.0 / (3.14159 * 1000.0);
const kInt16ToRpss: f32 = 1.0 / kRpssToInt16;

// Minimum scalar value for the linear and angular scalars
const kMinScalar: f32 = 0.872;

// Maximum scalar value for the linear and angular scalars
const kMaxScalar: f32 = 1.127;

// Enumerations for linear units used by the OTOS driver
const kLinearUnitMeters: u8 = 0;
const kLinearUnitInches: u8 = 1;

// Enumerations for angular units used by the OTOS driver
const kAngularUnitRadians: u8 = 0;
const kAngularUnitDegrees: u8 = 1;

pub struct OTOS {
    i2c: I2c,
    linear_unit: u8,
    angular_unit: u8,
    meter_to_unit: f32,
    rad_to_unit: f32,
}
impl OTOS {

    /// makes a new otos object
    /// the default address is 0x17
    pub fn new(address: u16, mut i2c: I2c) -> Self {
        let _ = i2c.set_slave_address(address);
        Self { i2c: i2c, linear_unit: kLinearUnitMeters, angular_unit: kAngularUnitDegrees, meter_to_unit: kMeterToInch, rad_to_unit: kRadianToDegree }
    }

    /// checks if the device is connected
    pub fn is_connected(&mut self) -> bool{
        let mut buf = [0];
        let prodid = self.i2c.block_read(kRegProductId, &mut buf);
        buf[0] == kProductId
    }

    /// initializes this device with default parameters
    pub fn begin(&mut self) -> Result<bool, Box<dyn Error>> {
        Ok(self.is_connected())
    }

    /// gets the hardware and firmware version numbers from the OTOS
    pub fn get_version_info(&mut self) -> Result<u16, Box<dyn Error>> {
        let mut buf: [u8; 2] = [0; 2];
        let _ = self.i2c.block_read(kRegHwVersion, &mut buf);
        Ok(u16::from_be_bytes(buf))
    }

    /// Performs a self-test on the OTOS
    pub fn self_test(&mut self) -> Result<bool, Box<dyn Error>> {
        let _ = self.i2c.write(&[kRegSelfTest, 0x01]);
        let mut reg_value = [0];
        for _i in 0..10 {
            sleep(Duration::from_secs_f32(0.005));
            let _ = self.i2c.block_read(kRegSelfTest, &mut reg_value);
            if ((reg_value[0] >> 1) & 0x01) == 0 {
                break;
            }
        }
        if ((reg_value[0] >> 2) & 0x01) == 1{
            Ok(true)
        }
        else {
            Ok(false)
        }
    }

    /// Calibrates the IMU on the OTOS, which removes the accelerometer and gyroscope offsets
    pub fn calibrate_imu(&mut self, num_samples: u8, wait_until_done: bool) -> Result<bool, Box<dyn Error>>{
        /*
        

        :param numSamples: Number of samples to take for calibration. Each
        sample takes about 2.4ms, so fewer samples can be taken for faster
        calibration, defaults to 255
        :type numSamples: int, optional
        :param waitUntilDone: hether to wait until the calibration is complete.
        Set false to calibrate asynchronously, see getImuCalibrationProgress(),
        defaults to True
        :type waitUntilDone: bool, optional
        :return: True if successful, otherwise False
        :rtype: bool
        */
        // Write the number of samples to the device
        let _ = self.i2c.write(&[kRegImuCalib, num_samples]);
        
        // Wait 1 sample period (2.4ms) to ensure the register updates
        sleep(Duration::from_secs_f32(0.003));

        // Do we need to wait until the calibration finishes?
        if !wait_until_done{
            return Ok(true);
        }
        
        // Wait for the calibration to finish, which is indicated by the IMU
        // calibration register reading zero, or until we reach the maximum
        // number of read attempts
        for _num_attempts in (1..num_samples).rev() {
            // Read the gryo calibration register value
            let mut calibration_value: [u8; 1] = [0];
            let _ = self.i2c.block_read(kRegImuCalib, &mut calibration_value);


            // Check if calibration is done
            if calibration_value[0] == 0{
                return Ok(true);
            }

            // Give a short delay between reads. As of firmware v1.0, samples take
            // 2.4ms each, so 3ms should guarantee the next sample is done. This
            // also ensures the max attempts is not exceeded in normal operation
            sleep(Duration::from_secs_f32(0.003));

        // Max number of attempts reached, calibration failed
        
        }
        Ok(false)
    }

    /// Gets the progress of the IMU calibration. Used for asynchronous calibration with calibrateImu()
    pub fn get_imu_calibration_process(&mut self) -> Result<u8, Box<dyn Error>> {
        let mut buf = [0];
        let _ = self.i2c.block_read(kRegImuCalib, &mut buf);
        Ok(buf[0])
    }

    /// Gets the linear unit used by all methods using a pose
    pub fn set_linear_unit(&mut self, unit: u8) -> Result<(), Box<dyn Error>> {
        if unit == self.linear_unit {
            return Ok(());
        }
        self.linear_unit = unit;
        if unit == kLinearUnitMeters {
            self.meter_to_unit = 1.0;
        }
        else {
            self.meter_to_unit = kMeterToInch;
        }
        Ok(())
    }

    /// Sets the angular unit used by all methods using a pose
    pub fn set_angular_unit(&mut self, unit: u8) -> Result<(), Box<dyn Error>> {
        if unit == self.angular_unit {
            return Ok(());
        }
        self.angular_unit = unit;
        if unit == kAngularUnitRadians {
            self.rad_to_unit = 1.0;
        }
        else {
            self.rad_to_unit = kRadianToDegree;
        }
        Ok(())
    }

    /// Gets the linear scalar used by the OTOS
    pub fn get_linear_scalar(&mut self) -> Result<f32, Box<dyn Error>>{
        let mut raw_scalar = [0];
        let _ = self.i2c.block_read(kRegScalarLinear, &mut raw_scalar);
        let scalar = if raw_scalar[0] > 127 {raw_scalar[0] as u16 - 256} else {raw_scalar[0] as u16};
        Ok(scalar as f32 * 0.001 + 1.0)
    }

    /// Sets the linear scalar used by the OTOS. Can be used to compensate for scaling issues with the sensor measurements
    pub fn set_linear_scalar(&mut self, scalar: f32) -> Result<bool, Box<dyn Error>>{
        if scalar < kMinScalar || scalar > kMaxScalar {
            return Ok(false);
        }
        let rawscalar = ((scalar - 1.0) * 1000.0) as u16;
        let _ = self.i2c.block_write(kRegScalarLinear, &rawscalar.to_be_bytes());
        Ok(true)
    }

    /// Gets the angular scalar used by the OTOS
    pub fn get_angular_scalar(&mut self) -> Result<f32, Box<dyn Error>> {
        let mut raw_scalar = [0];
        let _ = self.i2c.block_read(kRegScalarAngular, &mut raw_scalar);
        let scalar: u16 = if raw_scalar[0] > 127 {raw_scalar[0] as u16 - 256} else {raw_scalar[0] as u16};
        Ok(scalar as f32 * 0.001 + 1.0)
    }

    /// Sets the angular scalar used by the OTOS. Can be used to compensate for scaling issues with the sensor measurements
    pub fn set_angular_scalar(&mut self, scalar: f32) -> Result<bool, Box<dyn Error>> {
        if scalar < kMinScalar || scalar > kMaxScalar {
            return Ok(false);
        }
        let rawscalar = ((scalar - 1.0) * 1000.0) as u16;
        let _ = self.i2c.block_write(kRegScalarAngular, &rawscalar.to_be_bytes());
        Ok(true)
    }

    /// Resets the tracking algorithm, which resets the position to the origin, but can also be used to recover from some rare tracking errors
    pub fn reset_tracking(&mut self) -> Result<(), Box<dyn Error>> {
        let _ = self.i2c.block_write(kRegReset, &[0x01]);
        Ok(())
    }

    /// Gets the signal processing configuration from the OTOS
    pub fn get_signal_process_config(&mut self) -> Result<u8, Box<dyn Error>> {
        let mut buf = [0];
        let _ = self.i2c.block_read(kRegSignalProcess, &mut buf);
        Ok(buf[0])
    }

    /// Sets the signal processing configuration for the OTOS
    pub fn set_signal_process_config(&mut self, config: u8) -> Result<(), Box<dyn Error>> {
        let _ = self.i2c.block_write(kRegSignalProcess, &[config]);
        Ok(())
    }

    /// Gets the status register from the OTOS, which includes warnings and errors reported by the sensor
    pub fn get_status(&mut self) -> Result<u8, Box<dyn Error>> {
        let mut buf = [0];
        let _ = self.i2c.block_read(kRegStatus, &mut buf);
        Ok(buf[0])
    }

    /// Gets the offset of the OTOS
    pub fn get_offset(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegOffXL, kInt16ToMeter, kInt16ToRad))
    }

    /// Gets the offset of the OTOS. This is useful if your sensor is
    /// mounted off-center from a robot. Rather than returning the position of
    /// the sensor, the OTOS will return the position of the robot
    pub fn set_offset(&mut self, pose: Pose2d) -> Result<(), Box<dyn Error>> {
        self._write_pose_regs(kRegOffXL, pose, kMeterToInt16, kRadToInt16);
        Ok(())
    }

    /// Gets the position measured by the OTOS
    pub fn get_position(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegPosXL, kInt16ToMeter, kInt16ToRad))
    }

    /// Sets the position measured by the OTOS. This is useful if your
    /// robot does not start at the origin, or you have another source of
    /// location information (eg. vision odometry); the OTOS will continue
    /// tracking from this position
    pub fn set_position(&mut self, pose: Pose2d) -> Result<(), Box<dyn Error>> {
        let _ = self._write_pose_regs(kRegPosXL, pose, kMeterToInt16, kRadToInt16);
        Ok(())
    }

    /// Gets the velocity measured by the OTOS
    pub fn get_velocity(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegPosXL, kInt16ToMps, kInt16ToRps))
    }

    /// Gets the acceleration measured by the OTOS
    pub fn get_acceleration(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegAccXL, kInt16ToMpss, kInt16ToRpss))
    }

    /// Gets the standard deviation of the measured position
    pub fn get_position_stddev(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegPosStdXL, kInt16ToMeter, kInt16ToRad))
    }

    /// Gets the standard deviation of the measured velocity
    pub fn get_velocity_stddev(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegVelStdXL, kInt16ToMps, kInt16ToRps))
    }

    /// Gets the standard deviation of the measured acceleration
    pub fn get_acceleration_stddev(&mut self) -> Result<Pose2d, Box<dyn Error>> {
        Ok(self._read_pose_regs(kRegAccStdXL, kInt16ToMpss, kInt16ToRpss))
    }

    /// Gets the position, velocity, and acceleration measured by the
    /// OTOS in a single burst read
    pub fn get_pos_vel_acc(&mut self) -> Result<[Pose2d; 3], Box<dyn Error>> {
        let mut raw_data = [0; 18];
        let _ = self.i2c.block_read(kRegPosXL, &mut raw_data);
        let pos = self._regs_to_pose(&raw_data[0..6], kInt16ToMeter, kInt16ToRad);
        let vel = self._regs_to_pose(&raw_data[6..12], kInt16ToMps, kInt16ToRps);
        let acc = self._regs_to_pose(&raw_data[12..18], kInt16ToMpss, kInt16ToRpss);
        Ok([pos, vel, acc])
    }

    /// Gets the standard deviation of the measured position, velocity,
    /// and acceleration in a single burst read
    pub fn get_pos_vel_acc_stddev(&mut self) -> Result<[Pose2d; 3], Box<dyn Error>> {
        let mut raw_data = [0;18];
        let _ = self.i2c.block_read(kRegPosStdXL, &mut raw_data);
        let pos = self._regs_to_pose(&raw_data[0..6], kInt16ToMeter, kInt16ToRad);
        let vel = self._regs_to_pose(&raw_data[6..12], kInt16ToMps, kInt16ToRps);
        let acc = self._regs_to_pose(&raw_data[12..18], kInt16ToMpss, kInt16ToRpss);
        Ok([pos, vel, acc])
    }

    /// Gets the position, velocity, acceleration, and standard deviation
    /// of each in a single burst read
    pub fn get_pos_vel_acc_and_stddev(&mut self) -> Result<[Pose2d; 6], Box<dyn Error>> {
        let mut raw_data = [0;36];
        let _ = self.i2c.block_read(kRegPosStdXL, &mut raw_data);
        let pos = self._regs_to_pose(&raw_data[0..6], kInt16ToMeter, kInt16ToRad);
        let vel = self._regs_to_pose(&raw_data[6..12], kInt16ToMps, kInt16ToRps);
        let acc = self._regs_to_pose(&raw_data[12..18], kInt16ToMpss, kInt16ToRpss);
        let pos_std_dev = self._regs_to_pose(&raw_data[18..24], kInt16ToMeter, kInt16ToRad);
        let vel_std_dev = self._regs_to_pose(&raw_data[24..30], kInt16ToMps, kInt16ToRps);
        let acc_std_dev = self._regs_to_pose(&raw_data[30..36], kInt16ToMpss, kInt16ToRpss);
        Ok([pos, vel, acc, pos_std_dev, vel_std_dev, acc_std_dev])
    }

    /// Function to read raw pose registers and convert to specified units
    pub fn _read_pose_regs(&mut self, reg: u8, raw_to_xy: f32, raw_to_h: f32) -> Pose2d {
        let mut raw_data = [0; 6];
        let _ = self.i2c.block_read(reg, &mut raw_data);
        self._regs_to_pose(&raw_data, raw_to_xy, raw_to_h)
    }

    /// Function to write raw pose registers and convert from specified units
    pub fn _write_pose_regs(&mut self, reg: u8, pose: Pose2d, xy_to_raw: f32, h_to_raw: f32) {
        let raw_data = self._pose_to_regs(pose, xy_to_raw, h_to_raw);
        let _ = self.i2c.block_write(reg, &raw_data);
    }

    /// Function to convert raw pose registers to a pose structure
    pub fn _regs_to_pose(&mut self, raw_data: &[u8], raw_to_xy: f32, raw_to_h: f32) -> Pose2d {
        let raw_x = ((raw_data[1] as i32) << 8) | raw_data[0] as i32;
        let raw_y = ((raw_data[3] as i32) << 8) | raw_data[2] as i32;
        let raw_h = ((raw_data[5] as i32) << 8) | raw_data[4] as i32;
        let x2 = if raw_x > 32767 {raw_x - 65536} else {raw_x};
        let y2 = if raw_y > 32767 {raw_y - 65536} else {raw_y}; 
        let h2 = if raw_h > 32767 {raw_h - 65536} else {raw_h};
        let x = x2 as f32 * raw_to_xy * self.meter_to_unit;
        let y = y2 as f32 * raw_to_xy * self.meter_to_unit;
        let h = h2 as f32 * raw_to_h * self.rad_to_unit;
        Pose2d {x, y, h}
    }

    /// Function to convert a pose structure to raw pose registers
    pub fn _pose_to_regs(&mut self, pose: Pose2d, xy_to_raw: f32, h_to_raw: f32) -> [u8; 6] {
        let raw_x = (pose.x * xy_to_raw / self.meter_to_unit) as i16;
        let raw_y = (pose.y * xy_to_raw / self.meter_to_unit) as i16;
        let raw_h = (pose.h * h_to_raw / self.rad_to_unit) as i16;

        let mut raw_data = [0u8; 6];
        raw_data[0] = (raw_x & 0xFF) as u8;         // Lower byte of raw_x
        raw_data[1] = ((raw_x >> 8) & 0xFF) as u8;  // Upper byte of raw_x
        raw_data[2] = (raw_y & 0xFF) as u8;         // Lower byte of raw_y
        raw_data[3] = ((raw_y >> 8) & 0xFF) as u8;  // Upper byte of raw_y
        raw_data[4] = (raw_h & 0xFF) as u8;         // Lower byte of raw_h
        raw_data[5] = ((raw_h >> 8) & 0xFF) as u8;  // Upper byte of raw_h
        raw_data
    }
    
}