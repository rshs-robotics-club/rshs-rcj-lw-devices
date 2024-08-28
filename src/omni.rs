use rpi_build_hat_serial::motor_wrap::{Direction, Motor};
use rpi_build_hat_serial::raw::firmware::Port;
use std::error::Error;
use std::f32::consts::PI;
pub struct Omni {
    motor_a: Motor,
    motor_b: Motor,
    motor_c: Motor,
    motor_d: Motor,
}
impl Omni {
    /// creates a new omni object
    pub async fn new() -> Result<Self, Box<dyn Error>> {
        let m_a = Motor::new(Port::A, 1.0, Direction::Anticlockwise).await;
        let m_b = Motor::new(Port::B, 1.0, Direction::Anticlockwise).await;
        let m_c = Motor::new(Port::C, 1.0, Direction::Anticlockwise).await;
        let m_d = Motor::new(Port::D, 1.0, Direction::Anticlockwise).await;
        Ok(Self {
            motor_a: m_a,
            motor_b: m_b,
            motor_c: m_c,
            motor_d: m_d,
        })
    }

    /// creates a new omni object with custom ports
    pub async fn new_with_ports(a: Port, b: Port, c: Port, d: Port) -> Self {
        let m_a = Motor::new(a, 1.0, Direction::Anticlockwise).await;
        let m_b = Motor::new(b, 1.0, Direction::Anticlockwise).await;
        let m_c = Motor::new(c, 1.0, Direction::Anticlockwise).await;
        let m_d = Motor::new(d, 1.0, Direction::Anticlockwise).await;
        Self {
            motor_a: m_a,
            motor_b: m_b,
            motor_c: m_c,
            motor_d: m_d,
        }
    }

    /// creates a new omni object with custom limit
    pub async fn new_with_lim(limit: f32) -> Self {
        let m_a = Motor::new(Port::A, limit, Direction::Anticlockwise).await;
        let m_b = Motor::new(Port::B, limit, Direction::Anticlockwise).await;
        let m_c = Motor::new(Port::C, limit, Direction::Anticlockwise).await;
        let m_d = Motor::new(Port::D, limit, Direction::Anticlockwise).await;
        Self {
            motor_a: m_a,
            motor_b: m_b,
            motor_c: m_c,
            motor_d: m_d,
        }
    }

    pub fn find_rotated_point(x: f32, y: f32, degrees: f32) -> Result<[f32; 2], Box<dyn Error>> {
        let x2 = (x * f32::cos(degrees * PI / 180.0)) + (y * -f32::sin(degrees * PI / 180.0));
        let y2 = (x * f32::sin(degrees * PI / 180.0)) + (y * f32::cos(degrees * PI / 180.0));
        Ok([x2, y2])
    }

    /// runs motors with raw values (pid mode)
    pub async fn run_raw(&mut self, a_speed: f32, b_speed: f32, c_speed: f32, d_speed: f32) {
        self.motor_a.run_pid(a_speed).await;
        self.motor_b.run_pid(b_speed).await;
        self.motor_c.run_pid(c_speed).await;
        self.motor_d.run_pid(d_speed).await;
    }

    /// give the string without the port
    pub async fn run_custom_pid(&mut self, a_speed: f32, b_speed: f32, c_speed: f32, d_speed: f32, pid_string: String) {
        let a_pid = format!("pid {} {}", self.motor_a.port.clone() as u8, pid_string);
        let b_pid = format!("pid {} {}", self.motor_b.port.clone() as u8, pid_string);
        let c_pid = format!("pid {} {}", self.motor_c.port.clone() as u8, pid_string);
        let d_pid = format!("pid {} {}", self.motor_d.port.clone() as u8, pid_string);
        self.motor_a.run_pid_custom(a_speed, a_pid.clone()).await;
        self.motor_b.run_pid_custom(b_speed, b_pid.clone()).await;
        self.motor_c.run_pid_custom(c_speed, c_pid.clone()).await;
        self.motor_d.run_pid_custom(d_speed, d_pid.clone()).await;
    }
    /// runs motors with raw values ()
    pub async fn run_raw_pwm(&mut self, a_speed: f32, b_speed: f32, c_speed: f32, d_speed: f32) {
        self.motor_a.run_pwm(a_speed).await;
        self.motor_b.run_pwm(b_speed).await;
        self.motor_c.run_pwm(c_speed).await;
        self.motor_d.run_pwm(d_speed).await;
    }

    /// runs the robot with a vector, relative to the robot
    pub async fn move_xy_rel(
        &mut self,
        robot_speed: f32,
        x: f32,
        y: f32,
    ) -> Result<(), Box<dyn Error>> {
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_raw(a, b, c, d).await;
        Ok(())
    }

    /// runs the robot with a vector, relative to the robot
    pub async fn move_xy_rel_pwm(
        &mut self,
        robot_speed: f32,
        x: f32,
        y: f32,
    ) -> Result<(), Box<dyn Error>> {
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_raw_pwm(a, b, c, d).await;
        Ok(())
    }

    /// moves the robot with a vector.
    ///
    /// uses the pid mode to run the motors.
    /// # Parameters
    /// * robot_speed: the speed the motor would spin (-100.0 to 100.0)
    /// * gyro: a reference for using the gyro. (&Mpu6050)
    /// * code_speed: the amount of time in which the main code loops at. (secs)
    /// * x_abs: the x-value relative to the starting direction
    /// * y_abs: the y-value relative to the starting direction
    /// * face_angle: the angle in which the robot should face
    pub async fn move_xy(
        &mut self,
        robot_speed: f32,
        facing: f32,
        x_1: f32,
        y_1: f32,
        face_angle: f32,
        rotation_multiplier: f32,
    ) -> Result<(), Box<dyn Error>> {
        let rotated_point = Self::find_rotated_point(x_1, y_1, facing).unwrap();
        let (x, y) = (rotated_point[0], rotated_point[1]);
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        let rotation_factor = (facing + face_angle) * rotation_multiplier;
        a -= rotation_factor;
        b -= rotation_factor;
        c -= rotation_factor;
        d -= rotation_factor;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_raw(a, b, c, d).await;
        Ok(())
    }

    /// moves the robot with a vector.
    ///
    /// uses the pwm mode to run the motors
    /// # Parameters
    /// * robot_speed: the speed the motor would spin (-100.0 to 100.0)
    /// * gyro: a reference for using the gyro. (&Mpu6050)
    /// * code_speed: the amount of time in which the main code loops at. (secs)
    /// * x_abs: the x-value relative to the starting direction
    /// * y_abs: the y-value relative to the starting direction
    /// * face_angle: the angle in which the robot should face
    pub async fn move_xy_pwm(
        &mut self,
        robot_speed: f32,
        facing: f32,
        x_1: f32,
        y_1: f32,
        face_angle: f32,
        rotation_multiplier: f32,
    ) -> Result<(), Box<dyn Error>> {
        let rotated_point = Self::find_rotated_point(x_1, y_1, facing).unwrap();
        let (x, y) = (rotated_point[0], rotated_point[1]);
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        let rotation_factor = (facing + face_angle) * rotation_multiplier;
        a -= rotation_factor;
        b -= rotation_factor;
        c -= rotation_factor;
        d -= rotation_factor;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_raw_pwm(a, b, c, d).await;
        Ok(())
    }

    pub async fn move_xy_rotating(
        &mut self,
        robot_speed: f32,
        facing: f32,
        x_1: f32,
        y_1: f32,
        rotation_factor: f32,
    ) -> Result<(), Box<dyn Error>> {
        let rotated_point = Self::find_rotated_point(x_1, y_1, facing).unwrap();
        let (x, y) = (rotated_point[0], rotated_point[1]);
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        a -= rotation_factor;
        b -= rotation_factor;
        c -= rotation_factor;
        d -= rotation_factor;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_raw(a, b, c, d).await;
        Ok(())
    }
    pub async fn move_xy_custom_pid(
        &mut self,
        robot_speed: f32,
        facing: f32,
        x_1: f32,
        y_1: f32,
        face_angle: f32,
        rotation_multiplier: f32,
        pid_string: String,
    ) -> Result<(), Box<dyn Error>> {
        let rotated_point = Self::find_rotated_point(x_1, y_1, facing).unwrap();
        let (x, y) = (rotated_point[0], rotated_point[1]);
        let mut a: f32 = x + y;
        let mut b: f32 = x - y;
        let mut c: f32 = -x - y;
        let mut d: f32 = -x + y;
        let rotation_factor = (facing + face_angle) * rotation_multiplier;
        a -= rotation_factor;
        b -= rotation_factor;
        c -= rotation_factor;
        d -= rotation_factor;
        self.scale(robot_speed, &mut a, &mut b, &mut c, &mut d)
            .await;
        self.run_custom_pid(a, b, c, d, pid_string).await;
        Ok(())
    }
    /// moves the robot with angle starting from the vector [0, 1]
    pub async fn move_angle(
        &mut self,
        robot_speed: f32,
        facing: f32,
        move_angle: f32,
        face_angle: f32,
        rotation_multiplier: f32,
    ) -> Result<(), Box<dyn Error>> {
        let dir_x = Self::find_rotated_point(0.0, 1.0, move_angle).unwrap()[0];
        let dir_y = Self::find_rotated_point(0.0, 1.0, move_angle).unwrap()[1];
        let move_x = Self::find_rotated_point(dir_x, dir_y, facing).unwrap()[0];
        let move_y = Self::find_rotated_point(dir_x, dir_y, facing).unwrap()[1];
        let _ = self
            .move_xy(
                robot_speed,
                facing,
                move_x,
                move_y,
                face_angle,
                rotation_multiplier,
            )
            .await;
        Ok(())
    }

    /// moves the robot with angle starting from the vector [0, 1]
    pub async fn move_angle_pwm(
        &mut self,
        robot_speed: f32,
        facing: f32,
        move_angle: f32,
        face_angle: f32,
        rotation_multiplier: f32,
    ) -> Result<(), Box<dyn Error>> {
        let dir_x = Self::find_rotated_point(0.0, 1.0, move_angle).unwrap()[0];
        let dir_y = Self::find_rotated_point(0.0, 1.0, move_angle).unwrap()[1];
        let move_x = Self::find_rotated_point(dir_x, dir_y, facing).unwrap()[0];
        let move_y = Self::find_rotated_point(dir_x, dir_y, facing).unwrap()[1];
        let _ = self
            .move_xy_pwm(
                robot_speed,
                facing,
                move_x,
                move_y,
                face_angle,
                rotation_multiplier,
            )
            .await;
        Ok(())
    }

    /// stops the motors
    pub async fn stop(&mut self) -> Result<(), Box<dyn Error>> {
        self.run_raw_pwm(0.0, 0.0, 0.0, 0.0).await;
        Ok(())
    }

    /// scale 4 motor values
    pub async fn scale(
        &mut self,
        aim: f32,
        a_speed: &mut f32,
        b_speed: &mut f32,
        c_speed: &mut f32,
        d_speed: &mut f32,
    ) {
        let values = [a_speed.abs(), b_speed.abs(), c_speed.abs(), d_speed.abs()];
        let current_max: f32 = values
            .iter()
            .fold(f32::MIN, |a, b| if a > *b { a } else { *b });
        let multiplier: f32 = aim / current_max;
        *a_speed *= multiplier;
        *b_speed *= multiplier;
        *c_speed *= multiplier;
        *d_speed *= multiplier;
    }
}
