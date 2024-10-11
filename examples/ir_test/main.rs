use pollster;
use rshs_rcj_lw_devices::bbr_irseeker::{self, *};
use std::process::Command;
fn main() {
    let _ = pollster::block_on(async move{
        let mut ir = bbr_irseeker::Irseeker::new().await.unwrap();

        loop{
            let values = ir.get_both().await;
            println!("1: {}    2: {}", values[0], values[1]);
            //  let com = Command::new("espeak").arg(values[0].to_string()).output().expect("error");
        };
    });
}
