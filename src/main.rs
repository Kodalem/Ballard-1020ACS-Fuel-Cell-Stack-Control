#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use rp_pico::hal::bsp_pins;

fn current_purge_calculation(desired_current: u32) -> u32 {
    // Check if the current is under the fuel cell limit (75Amps)
    if desired_current <= 75 {
        let purge_time: u32 = (2300 * 1000) / desired_current;
        // Return the purge time
        return purge_time;
    } else {
        // Print an error message and throw an error
        crate::panic!("Desired current is over the fuel cell limit of 75Amps!");
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the relay control pin
    let mut relay_control_pin = pins.gpio22.into_push_pull_output();

    // Set the desired current (A)
    let desired_current: u32 = 50;
    // Set the purge setup time (ms)
    let purge_setup_time: u32 = 2000;
    // Set the purge duration, it should be less than 500ms
    let purge_duration: u32 = 500;

    // Print the purge valve control setup sequence
    info!("Purge valve control setup sequence");
    // Print the desired current
    info!("Desired current: {}", desired_current);
    // Calculate the purge time
    let purge_time: u32 = current_purge_calculation(desired_current);
    // Print the purge time
    info!("Purge time: {}", purge_time);

    // Print the purge valve startup sequence
    info!("Purge valve startup sequence");

    // Turn on the purge relay
    relay_control_pin.set_high().unwrap();
    // Print the purge relay is on
    info!("Purge relay is on");
    // Delay for the purge setup time
    delay.delay_ms(purge_setup_time);
    // Turn off the purge relay
    relay_control_pin.set_low().unwrap();
    // Print the purge relay is off
    info!("Purge relay is off");

    // Print the purge valve control runtime sequence and loop forever
    info!("Purge valve control runtime sequence");
    loop {
        // Turn on the purge relay
        relay_control_pin.set_high().unwrap();
        // Print the purge relay is on
        info!("Purge relay is on");
        // Delay for the purge duration
        delay.delay_ms(purge_duration);
        // Turn off the purge relay
        relay_control_pin.set_low().unwrap();
        // Print the purge relay is off
        info!("Purge relay is off");
        // Delay for the purge time
        delay.delay_ms(purge_time);
    }
}

// End of file
