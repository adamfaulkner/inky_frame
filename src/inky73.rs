use embedded_hal::{delay::DelayNs, digital::OutputPin};
use rp_pico::{
    hal::{
        self,
        gpio::{self, DynPinId},
        spi, Clock, Timer,
    },
    pac::{self, io_bank0, IO_BANK0, PADS_BANK0},
};
// Embed the `Hz` function/trait:
use fugit::RateExtU32;

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
}

impl Inky73 {
    pub fn new(sio: SIO, io_bank0: IO_BANK0, pads_bank0: PADS_BANK0) -> Inky73 {
        // The single-cycle I/O block controls our GPIO pins
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

        Inky73 {
            cs: pins
                .gpio17
                .into_push_pull_output_in_state(gpio::PinState::High)
                .into_dyn_pin(),
            dc: pins.gpio28.into_push_pull_output().into_dyn_pin(),
            reset: pins.gpio27.into_push_pull_output().into_dyn_pin(),
            spi,
            delay,
        }
    }

    pub fn reset(&mut self) -> Result<(), gpio::Error> {
        self.reset.set_high()?;
        self.delay.delay_ms(10);
        self.reset.set_low()?;
        self.delay.delay_ms(10);
        Ok(())
    }
}
