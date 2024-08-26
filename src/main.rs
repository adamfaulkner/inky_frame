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

use inky73::{Inky73, InkyPins};
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
#[allow(unused_imports)]
use panic_halt as _i;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{gpio, pac, spi, Clock, Timer};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

mod graphics;
mod inky73;
mod psram_display;
// Embed the `Hz` function/trait:
use fugit::RateExtU32;

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

    let delay = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let spi_rc = RefCell::new(spi);
    let inky_pins: InkyPins = InkyPins {
        gpio17: pins.gpio17.reconfigure(),
        gpio8: pins.gpio8.reconfigure(),
        gpio9: pins.gpio9.reconfigure(),
        gpio10: pins.gpio10.reconfigure(),
        gpio28: pins.gpio28.reconfigure(),
        gpio27: pins.gpio27.reconfigure(),
        gpio6: pins.gpio6.reconfigure(),
    };
    let mut inky = Inky73::new(&spi_rc, inky_pins, delay);

    inky.setup_and_status_loop();
}
