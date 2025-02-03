//! # Pico Countdown Blinky Example
//!
//! Blinks the LED on a Pico board, using an RP2040 Timer in Count-down mode.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use core::cell::RefCell;

use blink::{
    blink_signals, blink_signals_loop, BLINK_OK_LONG, BLINK_OK_SHORT_LONG,
    BLINK_OK_SHORT_SHORT_LONG,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::RefCellDevice;
use inky73::{Inky73, InkyPins};
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
#[allow(unused_imports)]
use panic_halt as _i;

use rp_pico::hal::gpio::bank0::{Gpio17, Gpio22};
use rp_pico::hal::gpio::{FunctionSioOutput, Pin, PullUp};
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{gpio, pac, spi, Clock, Timer};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

mod blink;
mod graphics;
mod inky73;
mod psram_display;
mod sdcard;
// Embed the `Hz` function/trait:
use fugit::RateExtU32;
use sdcard::InkySdCard;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio18.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio19.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio16.reconfigure();

    let spi: spi::Spi<
        spi::Disabled,
        pac::SPI0,
        (
            gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionSpi, gpio::PullNone>,
            gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSpi, gpio::PullUp>,
            gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSpi, gpio::PullNone>,
        ),
    > = spi::Spi::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        20.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let mut delay = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut led_pin = pins.gpio11.into_push_pull_output();

    let spi_rc = RefCell::new(spi);
    let inky_pins: InkyPins = InkyPins {
        gpio8: pins.gpio8.reconfigure(),
        gpio9: pins.gpio9.reconfigure(),
        gpio10: pins.gpio10.reconfigure(),
        gpio28: pins.gpio28.reconfigure(),
        gpio27: pins.gpio27.reconfigure(),
        gpio6: pins.gpio6.reconfigure(),
    };
    let frame_spi_cs: Pin<Gpio17, FunctionSioOutput, PullUp> = pins.gpio17.reconfigure();
    let frame_spi_device = RefCellDevice::new(&spi_rc, frame_spi_cs, delay).unwrap();

    let sdcard_cs: Pin<Gpio22, FunctionSioOutput, PullUp> = pins.gpio22.reconfigure();
    let sdcard_spi_device = RefCellDevice::new(&spi_rc, sdcard_cs, delay).unwrap();

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_SHORT_LONG);

    let mut inky = Inky73::new(frame_spi_device, inky_pins, delay);
    let mut sdcard = InkySdCard::new(sdcard_spi_device, delay, &mut led_pin);
    blink_signals_loop(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_SHORT_SHORT_LONG);

    inky.setup_and_status_loop(&input_buffer);
}
