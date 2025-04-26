use std::thread::sleep;
use std::time::Duration;
use std::{env, process::Command};
use std::string::String;
use rshs_rcj_lw_devices::button::Button;
use rshs_rcj_lw_devices::rpi_build_hat_serial::raw::firmware::send_ledmode;
use rshs_rcj_lw_devices::rpi_build_hat_serial::UART_SERIAL;
use rshs_rcj_lw_devices::rppal::i2c::I2c;
use rshs_rcj_lw_devices::pollster;
fn main() {
    let args: Vec<String> = env::args().collect();
    let com = &args[1];
    let arg = &args[2];

    let i2c = I2c::new().unwrap();
    Command::new("/home/robot/Desktop/buildHatBoot").spawn().unwrap().wait().unwrap();
    sleep(Duration::from_secs(10));
    let mut button = Button::new(i2c, 0x0c).unwrap();
    let _ = pollster::block_on(async move {
        let mut serial = UART_SERIAL.lock().await;
        send_ledmode(&mut serial, rshs_rcj_lw_devices::rpi_build_hat_serial::raw::firmware::LedMode::Both).await;
    });
    
    loop {
        Command::new(com).args([arg]).spawn().unwrap().wait().unwrap();
        println!("repeating...");
        sleep(Duration::from_secs(3));
    }
    
}
