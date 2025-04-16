#![no_std]
#![no_main]

mod fmt;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Timer;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    i2c,
    i2c::I2c,
    peripherals,
};
use fmt::info;

const STTS22H_ADDR: u8 = 0x70 >> 1; // Convert to a 7-bit address.

// const STTS22H_REG_WHOAMI: u8 = 0x01;
// const STTS22H_REG_TEMP_H_LIMIT: u8 = 0x02;
// const STTS22H_REG_TEMP_L_LIMIT: u8 = 0x03;
const STTS22H_REG_CTRL: u8 = 0x04;
// const STTS22H_REG_STATUS: u8 = 0x05;
const STTS22H_REG_TEMP_L_OUT: u8 = 0x06;
// const STTS22H_REG_TEMP_H_OUT: u8 = 0x07;

// LOW_ODR_START Enables 1 Hz ODR operating mode (see Section 11 Operating modes).
// BDU Default is set to 0 for BDU disabled; 1 for BDU enabled (if BDU is used, TEMP_L_OUT must be read  first).
// AVG[1:0] These bits are used to set the number of averages configuration. When in freerun mode, these bits also  set the ODR (see Table 13. Average configuration).
// IF_ADD_INC If this bit is set to 1, the automatic address increment is enabled when multiple I2C read and write  transactions are used.
// FREERUN Enables freerun mode
// TIME_OUT_DIS If this bit is set to 1, the timeout function of SMBus is disabled.  ONE_SHOT If this bit is set to 1, a new one-shot temperature acquisition is executed.
const STTS22H_CONFIG: u8 = 0b00111100;

static CURRENT_TEMP: Mutex<CriticalSectionRawMutex, f32> = Mutex::new(0.0);

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);
    // let mut blue_button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        embassy_stm32::time::Hertz(100_000),
        Default::default(),
    );

    info!("Hello, World!");
    led.set_low();

    info!("Getting initial STTS22H config");
    let mut ctrl_buffer = [0u8; 1];
    match i2c
        .write_read(STTS22H_ADDR, &[STTS22H_REG_CTRL], &mut ctrl_buffer)
        .await
    {
        Ok(()) => decode_ctrl(&ctrl_buffer),
        Err(e) => info!("I2C read error: {}", e),
    }

    info!("Initializing STTS22H via I2C...");
    match i2c
        .write(STTS22H_ADDR, &[STTS22H_REG_CTRL, STTS22H_CONFIG])
        .await
    {
        Ok(()) => info!("I2C configuration write successful"),
        Err(e) => info!("I2C write error setting configuration: {}", e),
    }

    info!("Confirming STTS22H config");
    // let mut ctrl_buffer = [0u8; 1];
    match i2c
        .write_read(STTS22H_ADDR, &[STTS22H_REG_CTRL], &mut ctrl_buffer)
        .await
    {
        Ok(()) => decode_ctrl(&ctrl_buffer),
        Err(e) => info!("I2C read error: {}", e),
    }

    spawner.spawn(track_temperature(i2c)).unwrap();

    loop {
        Timer::after_millis(750).await;

        led.set_high();
        let curr_temp = *CURRENT_TEMP.lock().await;
        info!(
            "Current temperature: {}°C ({}°F)",
            curr_temp,
            c_to_f(curr_temp)
        );
        led.set_low();
    }
}

#[embassy_executor::task]
async fn track_temperature(mut i2c: I2c<'static, embassy_stm32::mode::Async>) {
    let mut temperature_buffer = [0u8; 2];
    loop {
        match i2c
            .write_read(
                STTS22H_ADDR,
                &[STTS22H_REG_TEMP_L_OUT],
                &mut temperature_buffer,
            )
            .await
        {
            Ok(()) => {
                let temp = convert_temperature(&temperature_buffer);
                *CURRENT_TEMP.lock().await = temp;
            }
            Err(e) => info!("I2C read error: {}", e),
        }

        Timer::after_millis(500).await;
    }
}

fn convert_temperature(temperature_buffer: &[u8; 2]) -> f32 {
    let mut lsb: u16 = temperature_buffer[0] as u16 | ((temperature_buffer[1] as u16) << 8);
    if lsb > 2 ^ 15 {
        lsb -= 2 ^ 16
    }

    lsb as f32 / 100.0
}

fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

fn decode_ctrl(ctrl_buffer: &[u8; 1]) {
    let ctrl = ctrl_buffer[0];
    let low_odr_start = ctrl & (1 << 7) != 0;
    let bdu = ctrl & (1 << 6) != 0;
    let avg = (ctrl >> 4) & 0b11;
    let if_add_inc = ctrl & (1 << 3) != 0;
    let freerun = ctrl & (1 << 2) != 0;
    let time_out_dis = ctrl & (1 << 1) != 0;
    let one_shot = ctrl & 1 != 0;

    info!("STTS22H CTRL: low_odr_start={:?}, bdu={:?}, avg={}, if_add_inc={:?}, freerun={:?}, time_out_dis={:?}, one_shot={:?}", low_odr_start, bdu, avg, if_add_inc, freerun, time_out_dis, one_shot);
}
