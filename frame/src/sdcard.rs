use core::cell::RefCell;
use core::convert::Infallible;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rp_pico::hal::Timer;
use zune_core::bytestream::ZReaderTrait;

use crate::blink::{
    blink_signals_loop, BLINK_ERR_3_SHORT, BLINK_ERR_4_SHORT, BLINK_ERR_5_SHORT, BLINK_ERR_6_SHORT,
};

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

    pub fn get_len(&mut self) -> u32 {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let file = root_dir
            .open_file_in_dir("image.qoi", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        file.length()
    }

    pub fn read_image(&mut self, buf: &mut [u8; 4096], offset: usize) -> usize {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let mut file = root_dir
            .open_file_in_dir("image.qoi", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();

        file.seek_from_start(offset as u32);

        match file.read(buf) {
            Ok(read) => {
                return read;
            }
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
    }
}

pub struct SdCardReaderAdapter<'a, SPI: SpiDevice> {
    sd_card: RefCell<InkySdCard<'a, SPI>>,
    buf: RefCell<[u8; 4096]>,
}

impl<'a, SPI: SpiDevice> SdCardReaderAdapter<'a, SPI> {
    pub fn new(sd_card: InkySdCard<'a, SPI>) -> Self {
        SdCardReaderAdapter {
            sd_card: RefCell::new(sd_card),
            buf: RefCell::new([0; 4096]),
        }
    }
}

impl<'a, SPI: embedded_hal::spi::SpiDevice> ZReaderTrait for SdCardReaderAdapter<'a, SPI> {
    fn get_byte(&self, index: usize) -> Option<&u8> {
        let mut sd_card_borrow = self.sd_card.borrow_mut();
        {
            let mut b = self.buf.borrow_mut();
            sd_card_borrow.read_image(&mut b, index);
        }

        unsafe {
            return Some(&self.buf.as_ptr().as_ref().unwrap()[0]);
        }
    }

    fn get_slice(&self, index: core::ops::Range<usize>) -> Option<&[u8]> {
        assert!(index.end - index.start <= 4096);

        let mut sd_card_borrow = self.sd_card.borrow_mut();
        {
            let mut b = self.buf.borrow_mut();
            sd_card_borrow.read_image(&mut b, index.start);
        }
        unsafe {
            return Some(&self.buf.as_ptr().as_ref().unwrap()[0..(index.end - index.start)]);
        }
    }

    fn get_len(&self) -> usize {
        let mut sd_card_borrow = self.sd_card.borrow_mut();
        sd_card_borrow.get_len() as usize
    }
}
