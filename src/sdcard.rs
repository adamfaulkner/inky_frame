use core::convert::Infallible;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rp_pico::hal::Timer;

use crate::blink::{
    blink_signals_loop, BLINK_ERR_3_SHORT, BLINK_ERR_4_SHORT, BLINK_ERR_5_SHORT, BLINK_ERR_6_SHORT,
    BLINK_OK_LONG,
};
use crate::graphics::INPUT_BUFFER;

struct FakeTime {}

impl TimeSource for FakeTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_fat(0, 0)
    }
}

pub struct InkySdCard<'a, SPI: embedded_hal::spi::SpiDevice> {
    vmgr: VolumeManager<SdCard<SPI, Timer>, FakeTime>,
    led_pin: &'a mut dyn OutputPin<Error = Infallible>,
    delay: Timer,
}

impl<'a, SPI: embedded_hal::spi::SpiDevice> InkySdCard<'a, SPI> {
    pub fn new(
        spi_device: SPI,
        d: Timer,
        led_pin: &'a mut dyn OutputPin<Error = Infallible>,
    ) -> Self {
        let sdcard = SdCard::new(spi_device, d);
        InkySdCard {
            vmgr: VolumeManager::new(sdcard, FakeTime {}),
            delay: d,
            led_pin,
        }
    }

    pub fn read_image(&mut self, buf: &mut INPUT_BUFFER) {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let mut file = root_dir
            .open_file_in_dir("image.bmp", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();

        if file.is_eof() {
            blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT);
        }

        if file.length() > buf.len() as u32 {
            blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT);
        }
        match file.read(buf) {
            Ok(_) => (),
            Err(err) => match err {
                embedded_sdmmc::Error::DeviceError(_) => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT)
                }
                embedded_sdmmc::Error::FormatError(_) => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_4_SHORT)
                }
                embedded_sdmmc::Error::NoSuchVolume => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_5_SHORT)
                }
                embedded_sdmmc::Error::FilenameError(_) => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT)
                }
                embedded_sdmmc::Error::TooManyOpenVolumes => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_4_SHORT)
                }
                embedded_sdmmc::Error::TooManyOpenDirs => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_5_SHORT)
                }
                embedded_sdmmc::Error::TooManyOpenFiles => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT)
                }
                embedded_sdmmc::Error::BadHandle => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_4_SHORT)
                }
                embedded_sdmmc::Error::NotFound => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_5_SHORT)
                }
                embedded_sdmmc::Error::FileAlreadyOpen => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_3_SHORT)
                }
                embedded_sdmmc::Error::DirAlreadyOpen => {
                    blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_4_SHORT)
                }
                _ => blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_ERR_6_SHORT),
            },
        }
        blink_signals_loop(&mut self.led_pin, &mut self.delay, &BLINK_OK_LONG);
    }
}
