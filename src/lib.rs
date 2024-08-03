pub use rpi_build_hat_serial;
pub use rppal;
pub use lsm303dlhc;
pub mod bbr_irseeker;

pub use libm;
pub use nalgebra;

mod bits;
pub mod device;
pub mod mpu6050;
mod omni; // omni library is not done
pub mod compass;