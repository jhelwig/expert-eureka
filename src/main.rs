#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);
    let mut blue_button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    loop {
        info!("Hello, World!");
        blue_button.wait_for_any_edge().await;
        if blue_button.is_low() {
            led.set_high();
        } else {
            led.set_low();
        }
    }
}
