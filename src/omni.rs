use std::i32;

use rpi_build_hat_serial::motor_wrap::{Direction, Motor};
use rpi_build_hat_serial::raw::firmware::Port;
pub struct Omni {
    motor_a: Motor,
    motor_b: Motor,
    motor_c: Motor,
    motor_d: Motor,
}
impl Omni {
    /// creates a new omni object
    pub async fn new() -> Self {
        let m_a = Motor::new(Port::A, 1.0, Direction::Anticlockwise).await;
        let m_b = Motor::new(Port::B, 1.0, Direction::Anticlockwise).await;
        let m_c = Motor::new(Port::C, 1.0, Direction::Anticlockwise).await;
        let m_d = Motor::new(Port::D, 1.0, Direction::Anticlockwise).await;
        Self {
            motor_a: m_a,
            motor_b: m_b,
            motor_c: m_c,
            motor_d: m_d,
        }
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

    /// runs motors with raw values
    pub async fn run_raw(&mut self, a_speed: i8, b_speed: i8, c_speed: i8, d_speed: i8) {
        self.motor_a.run(a_speed).await;
        self.motor_b.run(b_speed).await;
        self.motor_c.run(c_speed).await;
        self.motor_d.run(d_speed).await;
    }

    /// scale 4 motor values
    async fn scale(
        &mut self,
        aim: f32,
        a_speed: &mut f32,
        b_speed: &mut f32,
        c_speed: &mut f32,
        d_speed: &mut f32,
    ) {
        let values = [a_speed.abs(), b_speed.abs(), c_speed.abs(), d_speed.abs()];
        let mut current_max: f32 = values
            .iter()
            .fold(f32::MIN, |a, b| if a > *b { a } else { *b });
        let multiplier: f32 = (aim / current_max);
        *a_speed *= multiplier;
        *b_speed *= multiplier;
        *c_speed *= multiplier;
        *d_speed *= multiplier;
    }
}
