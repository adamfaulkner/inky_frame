//! This file contains the implementation of the Inky73 drivers, including display, buttons, and
//! real time clock alarm.
//!
//! The bulk of this code was adapted from the Pimoroni inky frame drivers found here:
//! https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/inky73
//!
//! Specific sections that were copied directly are noted.
use core::{cell::RefCell, convert::Infallible};

use crate::{
    blink::{
        blink_codes::{BLINK_ERR_3_SHORT, BLINK_ERR_4_SHORT, BLINK_ERR_5_SHORT, BLINK_ERR_6_SHORT},
        blink_signals_loop,
    },
    sdcard::InkySdCard,
};
use cortex_m::{
    asm::nop,
    interrupt::{free, Mutex},
};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::{Operation, SpiDevice},
};
use fugit::ExtU32;
use rp_pico::{
    hal::{
        gpio::{
            self,
            bank0::{Gpio10, Gpio27, Gpio28, Gpio6, Gpio8, Gpio9},
            DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown,
        },
        pac::interrupt,
        timer::{Alarm, Alarm0},
        Timer,
    },
    pac,
};

// NOTE: this is duplicated with the constant in the converter package. Oh well.
pub const DISPLAY_BUFFER_SIZE: usize = 800 * 480 / 2;

/// The inky frame uses a shift register to collect the button states, the display "busy" state, and
/// the rtc alarm state. This shift register is very basic. Toggling latch to low begins reading the
/// register. Toggling clock to high advances the read by one bit. We can read bits on data.
pub struct ShiftRegister<C, L, D>
where
    C: OutputPin,
    L: OutputPin,
    D: InputPin,
{
    clock: C,
    latch: L,
    data: D,
}

impl<C, L, D> ShiftRegister<C, L, D>
where
    C: OutputPin,
    L: OutputPin,
    D: InputPin,
{
    fn new(clock: C, latch: L, data: D) -> Self {
        Self { clock, latch, data }
    }

    fn read(&mut self) -> u8 {
        self.latch.set_low().unwrap();
        nop();
        self.latch.set_high().unwrap();
        nop();

        let mut out: u8 = 0;
        for _ in 0..8 {
            out <<= 1;
            if self.data.is_high().unwrap() {
                out |= 1;
            }
            self.clock.set_high().unwrap();
            nop();
            self.clock.set_low().unwrap();
        }
        out
    }
}

/// Inky73 is a struct that represents the Inky73 device, including the display and the shift
/// register that manages the button states, display "busy" state, and rtc alarm state.
pub struct Inky73<Device>
where
    Device: SpiDevice,
{
    // Low = command mode, High = Data Mode
    dc: gpio::Pin<Gpio28, gpio::FunctionSioOutput, gpio::PullDown>,
    reset: gpio::Pin<Gpio27, gpio::FunctionSioOutput, gpio::PullDown>,
    spi_device: Device,
    delay: Timer,
    shift_register: ShiftRegister<
        gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioOutput, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSioOutput, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioInput, gpio::PullUp>,
    >,

    led_pin: gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::PullDown>,
    setup_complete: bool,
}

// These are commands for the display.
// All of this was copied from the Pimoroni inky frame drivers found here:
// https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/inky73
// Copied from inky73.cpp
#[allow(dead_code)]
mod registers {
    pub const PSR: u8 = 0x00;
    pub const PWR: u8 = 0x01;
    pub const POF: u8 = 0x02;
    pub const PFS: u8 = 0x03;
    pub const PON: u8 = 0x04;
    pub const BTST1: u8 = 0x05;
    pub const BTST2: u8 = 0x06;
    pub const DSLP: u8 = 0x07;
    pub const BTST3: u8 = 0x08;
    pub const DTM1: u8 = 0x10;
    pub const DSP: u8 = 0x11;
    pub const DRF: u8 = 0x12;
    pub const IPC: u8 = 0x13;
    pub const PLL: u8 = 0x30;
    pub const TSC: u8 = 0x40;
    pub const TSE: u8 = 0x41;
    pub const TSW: u8 = 0x42;
    pub const TSR: u8 = 0x43;
    pub const CDI: u8 = 0x50;
    pub const LPD: u8 = 0x51;
    pub const TCON: u8 = 0x60;
    pub const TRES: u8 = 0x61;
    pub const DAM: u8 = 0x65;
    pub const REV: u8 = 0x70;
    pub const FLG: u8 = 0x71;
    pub const AMV: u8 = 0x80;
    pub const VV: u8 = 0x81;
    pub const VDCS: u8 = 0x82;
    pub const T_VDCS: u8 = 0x84;
    pub const AGID: u8 = 0x86;
    pub const CMDH: u8 = 0xAA;
    pub const CCSET: u8 = 0xE0;
    pub const PWS: u8 = 0xE3;
    pub const TSSET: u8 = 0xE6;
}

/// This struct represents all of the pins used by the Inky73 display in addition to the spi device.
pub struct InkyPins {
    pub gpio8: Pin<Gpio8, FunctionSioOutput, PullDown>,
    pub gpio9: Pin<Gpio9, FunctionSioOutput, PullDown>,
    pub gpio10: Pin<Gpio10, FunctionSioInput, PullDown>,
    pub gpio28: Pin<Gpio28, FunctionSioOutput, PullDown>,
    pub gpio27: Pin<Gpio27, FunctionSioOutput, PullDown>,
    // This is an LED pin, we use it to report whenever the display is busy and to report various
    // error conditions.
    pub gpio6: Pin<Gpio6, FunctionSioOutput, PullDown>,
}

static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    free(|cs| {
        if let Some(alarm) = GLOBAL_ALARM.borrow(cs).borrow_mut().as_mut() {
            // Clear the interrupt flag for the alarm you used (e.g., Alarm0)
            alarm.clear_interrupt();
        }
    });
}

impl<Device> Inky73<Device>
where
    Device: SpiDevice,
{
    pub fn new(spi_device: Device, pins: InkyPins, mut delay: Timer) -> Self {
        let shift_register = ShiftRegister::new(
            pins.gpio8.into_push_pull_output(),
            pins.gpio9.into_push_pull_output(),
            pins.gpio10.into_pull_up_input(),
        );
        let mut alarm = delay.alarm_0().unwrap();
        alarm.enable_interrupt();

        free(|cs| {
            GLOBAL_ALARM.borrow(cs).replace(Some(alarm));
        });

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        }

        Inky73 {
            dc: pins.gpio28,
            reset: pins.gpio27,
            spi_device,
            delay,
            led_pin: pins.gpio6.into_push_pull_output().into_dyn_pin(),
            shift_register,
            setup_complete: false,
        }
    }

    pub fn reset(&mut self) -> Result<(), gpio::Error> {
        self.reset.set_low()?;
        self.delay.delay_ms(10);
        self.reset.set_high()?;
        self.delay.delay_ms(10);
        Ok(())
    }

    pub fn setup(&mut self) -> Result<(), gpio::Error> {
        self.reset()?;
        self.busy_wait();

        // Copied from inky73.cpp setup function.
        self.command(registers::CMDH, &[0x49, 0x55, 0x20, 0x08, 0x09, 0x18])?;
        self.command(registers::PWR, &[0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A])?;
        self.command(registers::PSR, &[0x53, 0x69])?;
        self.command(registers::PFS, &[0x00, 0x54, 0x00, 0x44])?;
        self.command(registers::BTST1, &[0x40, 0x1F, 0x1F, 0x2C])?;
        self.command(registers::BTST2, &[0x6F, 0x1F, 0x16, 0x25])?;
        self.command(registers::BTST3, &[0x6F, 0x1F, 0x1F, 0x22])?;
        self.command(registers::IPC, &[0x00, 0x04])?;
        self.command(registers::PLL, &[0x02])?;
        self.command(registers::TSE, &[0x00])?;
        self.command(registers::CDI, &[0x3F])?;
        self.command(registers::TCON, &[0x02, 0x00])?;
        self.command(registers::TRES, &[0x03, 0x20, 0x01, 0xE0])?;
        self.command(registers::VDCS, &[0x1E])?;
        self.command(registers::T_VDCS, &[0x00])?;
        self.command(registers::AGID, &[0x00])?;
        self.command(registers::PWS, &[0x2F])?;
        self.command(registers::CCSET, &[0x00])?;
        self.command(registers::TSSET, &[0x00])?;
        self.setup_complete = true;
        Ok(())
    }

    pub fn blink_err_code_loop(&mut self, sig: &[u8]) -> ! {
        blink_signals_loop(&mut self.led_pin, &mut self.delay, sig)
    }

    /// Display the image at the given index from the SD card.
    pub fn display_image_index<T: embedded_hal::spi::SpiDevice>(
        &mut self,
        sdcard: &mut InkySdCard<T>,
        index: usize,
    ) -> () {
        if !self.setup_complete {
            blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT)
        }

        self.led_pin.set_high().unwrap();

        let mut input_buffer = [0u8; DISPLAY_BUFFER_SIZE];
        match sdcard.read_file_with_index(index, &mut input_buffer) {
            Ok(_) => (),
            Err(0) => self.blink_err_code_loop(&BLINK_ERR_3_SHORT),
            Err(1) => self.blink_err_code_loop(&BLINK_ERR_3_SHORT),
            Err(2) => self.blink_err_code_loop(&BLINK_ERR_4_SHORT),
            Err(3) => self.blink_err_code_loop(&BLINK_ERR_3_SHORT),
            Err(4) => self.blink_err_code_loop(&BLINK_ERR_3_SHORT),
            // It's this one
            Err(5) => self.blink_err_code_loop(&BLINK_ERR_6_SHORT),
            Err(_) => self.blink_err_code_loop(&BLINK_ERR_3_SHORT),
        };

        let mut something_found = false;
        for b in input_buffer.iter() {
            if *b != 0 {
                something_found = true;
                break;
            }
        }

        if !something_found {
            self.blink_err_code_loop(&BLINK_ERR_5_SHORT)
        }

        match self.update(&input_buffer) {
            Ok(_) => {
                self.led_pin.set_low().unwrap();
            }
            Err(_) => blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT),
        }
    }

    pub fn is_busy(&mut self) -> bool {
        return (self.shift_register.read() & 128) == 0;
    }

    pub fn button_a_pressed(&mut self) -> bool {
        return (self.shift_register.read() & 1) == 1;
    }

    pub fn button_e_pressed(&mut self) -> bool {
        return (self.shift_register.read() & 16) == 16;
    }

    pub fn timer_went_off(&mut self) -> bool {
        return (self.shift_register.read() & 32) == 32;
    }

    // Delay implements a busy wait; we would like to not do that. This puts the CPU to sleep.
    fn efficient_sleep(&mut self, approx_ms: u32) {
        free(|cs| {
            GLOBAL_ALARM
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .schedule(approx_ms.millis())
                .unwrap()
        });
        // Actually put the processor to sleep.
        cortex_m::asm::wfi();
        // self.delay.delay_ms(approx_ms);
    }

    pub fn busy_wait(&mut self) {
        while self.is_busy() {
            self.efficient_sleep(500);
        }
    }

    pub fn command(&mut self, reg: u8, data: &[u8]) -> Result<(), Infallible> {
        self.dc.set_low()?;
        self.spi_device
            .transaction(&mut [Operation::Write(&[reg])])
            .unwrap();

        if data.len() > 0 {
            self.dc.set_high()?;
            self.spi_device
                .transaction(&mut [Operation::Write(data)])
                .unwrap();
        }
        Ok(())
    }

    pub fn update(
        &mut self,
        display_buffer: &[u8; DISPLAY_BUFFER_SIZE],
    ) -> Result<(), gpio::Error> {
        self.setup()?;

        self.dc.set_low()?; // command mode
        self.spi_device
            .transaction(&mut [Operation::Write(&[registers::DTM1])])
            .unwrap();
        self.dc.set_high()?; // data mode

        self.spi_device
            .transaction(&mut [Operation::Write(display_buffer)])
            .unwrap();

        self.dc.set_low()?; // end data mode

        self.busy_wait();

        self.command(registers::PON, &[0])?; // turn on
        self.busy_wait();

        self.command(registers::DRF, &[0])?; // start display refresh
        self.busy_wait();
        self.command(registers::POF, &[])?;
        self.busy_wait();
        Ok(())
    }
}
