use core::{cell::RefCell, convert::Infallible};

use cortex_m::asm::nop;

use crate::{
    blink::{
        blink_signals, blink_signals_loop, BLINK_ERR_3_SHORT, BLINK_ERR_4_SHORT, BLINK_ERR_5_SHORT,
        BLINK_ERR_6_SHORT, BLINK_OK_LONG, BLINK_OK_SHORT_LONG, BLINK_OK_SHORT_SHORT_LONG,
    },
    sdcard::InkySdCard,
};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::{Operation, SpiBus, SpiDevice},
};
use embedded_hal_bus::spi::RefCellDevice;
use rp_pico::hal::{
    gpio::{
        self,
        bank0::{Gpio10, Gpio17, Gpio27, Gpio28, Gpio6, Gpio8, Gpio9},
        DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullUp,
    },
    Timer,
};

// Dimensions: 800 x 480
// Each pixel is one nibble
// Black is 0

pub const DISPLAY_WIDTH: usize = 800;
pub const DISPLAY_HEIGHT: usize = 480;
pub const DISPLAY_BUFFER_SIZE: usize = 800 * 480 / 2;

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

// Copied from inky73.cpp
const PSR: u8 = 0x00;
const PWR: u8 = 0x01;
const POF: u8 = 0x02;
const PFS: u8 = 0x03;
const PON: u8 = 0x04;
const BTST1: u8 = 0x05;
const BTST2: u8 = 0x06;
const DSLP: u8 = 0x07;
const BTST3: u8 = 0x08;
const DTM1: u8 = 0x10;
const DSP: u8 = 0x11;
const DRF: u8 = 0x12;
const IPC: u8 = 0x13;
const PLL: u8 = 0x30;
const TSC: u8 = 0x40;
const TSE: u8 = 0x41;
const TSW: u8 = 0x42;
const TSR: u8 = 0x43;
const CDI: u8 = 0x50;
const LPD: u8 = 0x51;
const TCON: u8 = 0x60;
const TRES: u8 = 0x61;
const DAM: u8 = 0x65;
const REV: u8 = 0x70;
const FLG: u8 = 0x71;
const AMV: u8 = 0x80;
const VV: u8 = 0x81;
const VDCS: u8 = 0x82;
const T_VDCS: u8 = 0x84;
const AGID: u8 = 0x86;
const CMDH: u8 = 0xAA;
const CCSET: u8 = 0xE0;
const PWS: u8 = 0xE3;
const TSSET: u8 = 0xE6;

pub struct InkyPins {
    pub gpio8: Pin<Gpio8, FunctionSioOutput, PullDown>,
    pub gpio9: Pin<Gpio9, FunctionSioOutput, PullDown>,
    pub gpio10: Pin<Gpio10, FunctionSioInput, PullDown>,
    pub gpio28: Pin<Gpio28, FunctionSioOutput, PullDown>,
    pub gpio27: Pin<Gpio27, FunctionSioOutput, PullDown>,
    pub gpio6: Pin<Gpio6, FunctionSioOutput, PullDown>,
}

impl<Device> Inky73<Device>
where
    Device: SpiDevice,
{
    pub fn new(spi_device: Device, pins: InkyPins, delay: Timer) -> Self {
        let shift_register = ShiftRegister::new(
            pins.gpio8.into_push_pull_output(),
            pins.gpio9.into_push_pull_output(),
            pins.gpio10.into_pull_up_input(),
        );

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

        // Copied from inky73.cpp
        self.command(CMDH, &[0x49, 0x55, 0x20, 0x08, 0x09, 0x18])?;
        self.command(PWR, &[0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A])?;
        self.command(PSR, &[0x53, 0x69])?;
        //self.command(PSR, &[0x5F, 0x69])?;
        self.command(PFS, &[0x00, 0x54, 0x00, 0x44])?;
        self.command(BTST1, &[0x40, 0x1F, 0x1F, 0x2C])?;
        self.command(BTST2, &[0x6F, 0x1F, 0x16, 0x25])?;
        self.command(BTST3, &[0x6F, 0x1F, 0x1F, 0x22])?;
        self.command(IPC, &[0x00, 0x04])?;
        self.command(PLL, &[0x02])?;
        self.command(TSE, &[0x00])?;
        self.command(CDI, &[0x3F])?;
        self.command(TCON, &[0x02, 0x00])?;
        self.command(TRES, &[0x03, 0x20, 0x01, 0xE0])?;
        self.command(VDCS, &[0x1E])?;
        self.command(T_VDCS, &[0x00])?;
        self.command(AGID, &[0x00])?;
        self.command(PWS, &[0x2F])?;
        self.command(CCSET, &[0x00])?;
        self.command(TSSET, &[0x00])?;
        self.setup_complete = true;
        Ok(())
    }

    pub fn blink_err_code_loop(&mut self, sig: &[u8]) -> ! {
        blink_signals_loop(&mut self.led_pin, &mut self.delay, sig)
    }

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

    pub fn busy_wait(&mut self) {
        while self.is_busy() {
            // TODO: can we do something smarter here
            nop();
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
            .transaction(&mut [Operation::Write(&[DTM1])])
            .unwrap();
        self.dc.set_high()?; // data mode

        self.spi_device
            .transaction(&mut [Operation::Write(display_buffer)])
            .unwrap();

        self.dc.set_low()?; // end data mode

        self.busy_wait();

        self.command(PON, &[0])?; // turn on
        self.busy_wait();

        self.command(DRF, &[0])?; // start display refresh
        self.busy_wait();
        self.command(POF, &[])?;
        self.busy_wait();
        Ok(())
    }
}
