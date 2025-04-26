use std::thread::{sleep, Thread};
use std::time::Duration;

use rpi_build_hat_serial::raw::bootloader::*;
use rpi_build_hat_serial::raw::*;
use rpi_build_hat_serial::raw::firmware::{send_ledmode, send_version};
use rpi_build_hat_serial::*;
use pollster;
fn main() {
    unsafe {
        let _ = pollster::block_on(async move {
            let mut serial = UART_SERIAL.lock().await;
            let firmware_len = FIRMWARE.len();
            let _ = send_clear(&mut serial).await;
            println!("sent clear");
            let _ = send_load(&mut serial, firmware_len.to_string().as_str(), checksum(&FIRMWARE), &FIRMWARE).await;
            println!("sent load");
            let _ = send_signature(&mut serial, SIGNATURE.len().to_string().as_str(), &SIGNATURE).await;
            println!("sent signature");
            
            let _ = send_verify(&mut serial).await;
            println!("sent verify");
            let _ = send_reboot(&mut serial).await;
            println!("sent reboot");
        });
    }
}
// use pollster;
// use rpi_build_hat_serial::raw::bootloader::*;
// use rpi_build_hat_serial::raw::firmware::{send_help, send_ledmode};
// use rpi_build_hat_serial::raw::*;
// use rpi_build_hat_serial::*;
// fn main() {
//     unsafe {
//         let _ = pollster::block_on(async move {
//             let mut serial = UART_SERIAL.lock().await;
//             let firmware_len = FIRMWARE.len();
//             println!("will be sending firmware with len {firmware_len}");
//             let _ = send_load(
//                 &mut serial,
//                 firmware_len.to_string().as_str(),
//                 // &transform_u32_to_array_of_u8(checksum(&FIRMWARE)),
//                 checksum(&FIRMWARE),
//                 &FIRMWARE,
//             )
//             .await;
//             println!("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ load sent");
//             let _ = send_signature(
//                 &mut serial,
//                 SIGNATURE.len().to_string().as_str(),
//                 &SIGNATURE,
//             ) 
//             .await;
//             println!("^^^^^^^^^^^^^^^^^^^^^^^^^^^^ signature sent");
//             let _ = send_verify(&mut serial).await;
//             println!("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ verify sent");
//             // let _ = skip_verify_success(&mut serial).await;
//             let _ = send_reboot(&mut serial).await;
//             println!("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ reboot sent");
//             // build_hat::raw::send_help(&mut serial).await;
//             println!("enter read loop");
//             loop {
//                 let s = read_line(&mut serial).await;
//                 if let Ok(s) = s {
//                     let s = s.strip_suffix("\r\n").unwrap_or(&s);
//                     // println!("ok well {s}");
//                     println!("-->{s}<--");
//                 }
//             }
//         });
//     }
// }