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

use inky73::Inky73;
// The macro for our start-up function
use rp_pico::entry;

use cortex_m::prelude::*;

// GPIO traits
use embedded_hal::digital::OutputPin;

// Traits for converting integers to amounts of time
use fugit::ExtU32;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
#[allow(unused_imports)]
use panic_halt as _i;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

mod graphics;
mod inky73;
mod psram_display;

#[entry]
fn main() -> ! {
    let mut inky = Inky73::new();
    inky.setup_and_status_loop();
}
