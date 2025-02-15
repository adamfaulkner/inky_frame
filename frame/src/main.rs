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

use blink::BLINK_ERR_3_SHORT;
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

use rp_pico::hal::gpio::bank0::{Gpio17, Gpio2, Gpio22};
use rp_pico::hal::gpio::{FunctionSioOutput, Pin, PullUp};
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::{gpio, pac, spi, Clock, Timer};

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

mod blink;
mod inky73;
mod psram_display;
mod rtc;
mod sdcard;
// Embed the `Hz` function/trait:
use fugit::RateExtU32;
use rtc::Pcf85063a;
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

    // We want to hold it awake until we finish the transaction..
    let mut hold_awake_pin: Pin<Gpio2, FunctionSioOutput, PullUp> = pins.gpio2.reconfigure();
    hold_awake_pin.set_high().unwrap();

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
    let mut sdcard_led_pin = pins.gpio12.into_push_pull_output();

    // Configure the external real time clock, which uses i2c
    let sda: Pin<gpio::bank0::Gpio4, gpio::FunctionI2c, gpio::PullUp> = pins.gpio4.reconfigure();
    let scl: Pin<gpio::bank0::Gpio5, gpio::FunctionI2c, gpio::PullUp> = pins.gpio5.reconfigure();
    let mut i2c = rp_pico::hal::I2C::i2c0(
        pac.I2C0,
        sda,
        scl,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut rtc = Pcf85063a::new(&mut i2c);

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

    let mut inky = Inky73::new(frame_spi_device, inky_pins, delay);
    let mut sdcard = InkySdCard::new(sdcard_spi_device, delay, &mut sdcard_led_pin);

    match inky.setup() {
        Ok(_) => (),
        Err(_) => inky.blink_err_code_loop(&BLINK_ERR_3_SHORT),
    };

    let mut ready_to_draw = true;
    loop {
        let mut image_idx = rtc.get_ram_byte();
        if inky.button_a_pressed() {
            image_idx -= 1;
            rtc.put_ram_byte(image_idx);
            ready_to_draw = true;
        }

        if inky.button_e_pressed() {
            image_idx += 1;
            rtc.put_ram_byte(image_idx);
            ready_to_draw = true;
        }

        if inky.timer_went_off() {
            image_idx += 1;
            rtc.put_ram_byte(image_idx);
            ready_to_draw = true;
            rtc.clear_timer_flag();
        }

        // This ready_to_draw stuff is only relevant on USB power, as power is withdrawn after one iteration of this loop.
        if ready_to_draw {
            inky.display_image_index(&mut sdcard, image_idx as usize);
            ready_to_draw = false;
            // Set the timer so that we'll wake if on battery, and advance the draw index.
            rtc.set_minutes_timer(5);
        }

        // Sleep now if on battery
        hold_awake_pin.set_low().unwrap();
        delay.delay_ms(100);
    }
}
