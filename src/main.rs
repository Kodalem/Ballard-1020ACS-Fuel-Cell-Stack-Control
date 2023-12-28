#![no_std]
#![no_main]

use bme680::*;
use bsp::entry;
use core::fmt;
use core::fmt::Write;
use core::time::Duration;
use defmt::*;
use defmt_rtt as _;

use embedded_hal::blocking::delay;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;
// Pull in any important traits
use rp_pico::hal::prelude::*;

use cortex_m::delay::Delay;
use hal::gpio::Pins;
use hal::multicore::{Multicore, Stack};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
};
use rp_pico::hal::bsp_pins;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static
/// values is reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything seperately
/// and modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte
/// alignment, which allows the stack guard to take up the least amount of
/// usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

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
    let mut sio = Sio::new(pac.SIO);

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

    // Multi-core setup
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );
    let mut uart = bsp::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio20.into_function::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio21.into_function::<hal::gpio::FunctionI2C>();

    info!("I2C pins configured");

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    info!("I2C driver created");

    info!("Initializing BME680 sensor");
    // TODO - Figure out the hanging of the system when initializing the sensor
    let mut dev = Bme680::init(i2c, &mut delay, I2CAddress::Primary).unwrap();
    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS2x)
        .with_temperature_oversampling(OversamplingSetting::OS4x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();

    info!("Setting sensor settings");
    dev.set_sensor_settings(&mut delay, settings).unwrap();
    let profile_duration = dev.get_profile_dur(&settings.0).unwrap();

    // Read sensor data
    info!("Reading sensor data");
    dev.set_sensor_mode(&mut delay, PowerMode::ForcedMode);
    let sensor_settings = dev.get_sensor_settings(settings.1);

    info!("Retrieving sensor data");
    let (data, _state) = dev.get_sensor_data(&mut delay).unwrap();
    info!("Temperature {}°C", data.temperature_celsius());
    info!("Pressure {}hPa", data.pressure_hpa());
    info!("Humidity {}%", data.humidity_percent());
    info!("Gas Resistence {}Ω", data.gas_resistance_ohm());

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

    uart.write_full_blocking(b"UART example\r\n");

    let mut value = 0u32;

    // Start the second core running the communication and temperature measurement loop
    info!("Starting core 1");
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // Get the second core's copy of the `CorePeripherals`, which are per-core.
            // Unfortunately, `cortex-m` doesn't support this properly right now,
            // so we have to use `steal`.
            let core = unsafe { pac::CorePeripherals::steal() };
            // Loop the temperature measurement and communication
            loop {}
        })
        .unwrap();

    // Print the purge valve control runtime sequence and loop forever
    info!("Purge valve control runtime sequence");
    loop {
        // Turn on the purge relay
        relay_control_pin.set_high().unwrap();
        // Print the purge relay is on
        info!("Purge relay is on");
        // Send data over UART that the purge relay is on
        writeln!(uart, "ON\r\n").unwrap();
        // Delay for the purge duration
        delay.delay_ms(purge_duration);

        // Turn off the purge relay
        relay_control_pin.set_low().unwrap();
        // Print the purge relay is off
        info!("Purge relay is off");
        // Send data over UART that the purge relay is off
        writeln!(uart, "OFF\r\n").unwrap();
        // Delay for the purge time
        delay.delay_ms(purge_time);
    }
}

// End of file
