use cortex_m::asm::nop;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiBus,
};
use rp_pico::{
    hal::{
        self,
        gpio::{self, DynPinId},
        spi, Clock, Timer,
    },
    pac,
};
// Embed the `Hz` function/trait:
use fugit::RateExtU32;

// Dimensions: 800 x 480
// Each pixel is one nibble
// Black is 0

const DISPLAY_WIDTH: usize = 800;
const DISPLAY_HEIGHT: usize = 480;
const DISPLAY_BUFFER_SIZE: usize = 800 * 480 / 2;

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

pub struct Inky73 {
    // Low = select the display, High = do not
    cs: gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::PullDown>,
    // Low = command mode, High = Data Mode
    dc: gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::PullDown>,
    reset: gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::PullDown>,
    spi: spi::Spi<
        spi::Enabled,
        pac::SPI0,
        (
            gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionSpi, gpio::PullNone>,
            gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSpi, gpio::PullUp>,
            gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSpi, gpio::PullNone>,
        ),
    >,
    delay: Timer,
    shift_register: ShiftRegister<
        gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioOutput, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSioOutput, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioInput, gpio::PullUp>,
    >,

    led_pin: gpio::Pin<DynPinId, gpio::FunctionSioOutput, gpio::PullDown>,
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

impl Inky73 {
    pub fn new() -> Inky73 {
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

        let shift_register = ShiftRegister::new(
            pins.gpio8.into_push_pull_output(),
            pins.gpio9.into_push_pull_output(),
            pins.gpio10.into_pull_up_input(),
        );

        Inky73 {
            cs: pins
                .gpio17
                .into_push_pull_output_in_state(gpio::PinState::High)
                .into_dyn_pin(),
            dc: pins.gpio28.into_push_pull_output().into_dyn_pin(),
            reset: pins
                .gpio27
                .into_push_pull_output_in_state(gpio::PinState::High)
                .into_dyn_pin(),
            spi,
            delay,
            led_pin: pins.gpio6.into_push_pull_output().into_dyn_pin(),
            shift_register,
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
        self.command(TSSET, &[0x00])
    }

    pub fn setup_and_status_loop(&mut self) -> ! {
        match self.setup() {
            Ok(_) => (),
            Err(_) => blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT),
        }

        blink_signals(&mut self.led_pin, &mut self.delay, &BLINK_OK_LONG);
        blink_signals(&mut self.led_pin, &mut self.delay, &BLINK_OK_LONG);

        match self.update() {
            Ok(_) => blink_signals_loop(
                &mut self.led_pin,
                &mut self.delay,
                &BLINK_OK_SHORT_SHORT_LONG,
            ),
            Err(_) => blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT),
        }
    }

    pub fn is_busy(&mut self) -> bool {
        return (self.shift_register.read() & 128) == 0;
    }

    pub fn busy_wait(&mut self) {
        while self.is_busy() {
            nop();
        }
    }

    pub fn command(&mut self, reg: u8, data: &[u8]) -> Result<(), gpio::Error> {
        self.cs.set_low()?;
        self.dc.set_low()?;

        self.spi.write(&[reg])?;

        if data.len() > 0 {
            self.spi.flush()?;
            self.dc.set_high()?;
            self.spi.write(data)?;
        }
        self.spi.flush()?;
        self.cs.set_high()
    }

    pub fn update(&mut self) -> Result<(), gpio::Error> {
        self.setup()?;

        self.cs.set_low()?;
        self.dc.set_low()?; // command mode
        self.spi.write(&[DTM1])?;
        self.spi.flush()?;

        self.dc.set_high()?; // data mode

        let mut display_buffer: [u8; DISPLAY_BUFFER_SIZE] = [0; DISPLAY_BUFFER_SIZE];

        let circle_radius = DISPLAY_HEIGHT / 2;

        for i in 0..DISPLAY_HEIGHT {
            for j in 0..DISPLAY_WIDTH / 2 {
                let j1 = j * 2;
                let j2 = j1 + 1;
                let v1: u8 = if ((i * i) + (j1 * j1)) < circle_radius * circle_radius {
                    0 // BLACK
                } else {
                    ((i / 60) % 8) as u8
                };
                let v2: u8 = if ((i * i) + (j2 * j2)) < circle_radius * circle_radius {
                    0 // BLACK
                } else {
                    ((i / 60) % 8) as u8
                };

                let v: u8 = (v1 << 4) | v2;

                let coord = (i * DISPLAY_WIDTH / 2) + j;
                if coord >= DISPLAY_BUFFER_SIZE {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_4_SHORT);
                }
                display_buffer[coord] = v;
            }
        }

        self.spi.write(&display_buffer)?;
        self.spi.flush()?;

        /* Here is where we would write our graphics data to the screen.
        uint totalLength = 0;
        gpio_put(CS, 1);
        graphics->frame_convert(PicoGraphics::PEN_INKY7, [this, &totalLength](void *buf, size_t length) {
          if (length > 0) {
            gpio_put(CS, 0);
            spi_write_blocking(spi, (const uint8_t*)buf, length);
            totalLength += length;
            gpio_put(CS, 1);
          }
        });
        */

        self.dc.set_low()?; // end data mode
        self.cs.set_high()?;

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

// Setup some blinking codes:
const BLINK_OK_LONG: [u8; 1] = [8u8];
const BLINK_OK_SHORT_LONG: [u8; 4] = [1u8, 0u8, 6u8, 0u8];
const BLINK_OK_SHORT_SHORT_LONG: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_ERR_3_SHORT: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_4_SHORT: [u8; 8] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_5_SHORT: [u8; 10] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_6_SHORT: [u8; 12] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];

fn blink_signals(
    pin: &mut dyn embedded_hal::digital::OutputPin<Error = core::convert::Infallible>,
    delay: &mut dyn DelayNs,
    sig: &[u8],
) {
    for bit in sig {
        if *bit != 0 {
            pin.set_high().unwrap();
        } else {
            pin.set_low().unwrap();
        }

        let length = if *bit > 0 { *bit } else { 1 };

        for _ in 0..length {
            delay.delay_ms(100);
        }
    }

    pin.set_low().unwrap();

    delay.delay_ms(500);
}

fn blink_signals_loop(
    pin: &mut dyn embedded_hal::digital::OutputPin<Error = core::convert::Infallible>,
    delay: &mut dyn DelayNs,
    sig: &[u8],
) -> ! {
    loop {
        blink_signals(pin, delay, sig);
        delay.delay_ms(1000);
    }
}
