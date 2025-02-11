use core::cell::RefCell;
use core::convert::Infallible;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use embedded_sdmmc::{File, SdCard, TimeSource, Timestamp, Volume, VolumeIdx, VolumeManager};
use rp_pico::hal::Timer;

use crate::{
    blink::{
        blink_signals_loop, BLINK_ERR_3_SHORT, BLINK_ERR_4_SHORT, BLINK_ERR_5_SHORT,
        BLINK_ERR_6_SHORT,
    },
    inky73::DISPLAY_BUFFER_SIZE,
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

    pub fn get_number_of_files(&mut self) -> usize {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();
        let mut root_dir = volume.open_root_dir().unwrap();
        // This is stupid, whatever.
        let mut num_files = 0;
        root_dir
            .iterate_dir(|f| {
                if f.size == 0 {
                    return;
                }
                num_files += 1;
            })
            .unwrap();
        num_files
    }

    pub fn blink_err_code_loop(&mut self, signal: &[u8]) -> ! {
        blink_signals_loop(self.led_pin, &mut self.delay, signal)
    }

    pub fn read_file_with_index(
        &mut self,
        index: usize,
        buf: &mut [u8; DISPLAY_BUFFER_SIZE],
    ) -> Result<(), usize> {
        let real_index = index % self.get_number_of_files();

        let mut volume = match self.vmgr.open_volume(VolumeIdx(0)) {
            Ok(x) => x,
            Err(_) => return Err(0),
        };

        let mut root_dir = volume.open_root_dir().unwrap();

        // This is stupid, whatever.

        let mut file_idx = 0;
        let mut chosen_name = None;
        match root_dir.iterate_dir(|f| {
            // The sdcard library has a weird habit of including the volume label inside root_dir.iterate_dir
            if f.size == 0 {
                return;
            }
            if file_idx == real_index {
                chosen_name = Some(f.name.clone());
            }

            file_idx += 1;
        }) {
            Ok(_) => (),
            Err(_) => return Err(2),
        };

        let read_bytes = {
            let mut file = match root_dir
                .open_file_in_dir(chosen_name.clone().unwrap(), embedded_sdmmc::Mode::ReadOnly)
            {
                Ok(x) => x,
                Err(_) => return Err(3),
            };

            file.seek_from_start(0u32).unwrap();
            let read_bytes = match file.read(buf) {
                Ok(x) => x,
                Err(_) => return Err(4),
            };
            if read_bytes != buf.len() {
                // return Err(5);
            }
            read_bytes
        };

        let mut log_file = root_dir
            .open_file_in_dir("log", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)
            .unwrap();

        log_file.write(b"hello\n").unwrap();
        log_file
            .write(chosen_name.clone().unwrap().base_name())
            .unwrap();
        log_file.write(b"\n").unwrap();
        log_file.write(&read_bytes.to_be_bytes()).unwrap();
        log_file.close().unwrap();
        Ok(())
    }

    pub fn get_len(&mut self) -> u32 {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let file = root_dir
            .open_file_in_dir("b.out", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        file.length()
    }

    /*
    pub fn read_image(&mut self, buf: &mut [u8; DISPLAY_BUFFER_SIZE], index: usize) -> usize {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let mut file = root_dir
            .open_file_in_dir("b.out", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();

        file.seek_from_start(0u32).unwrap();

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
    */
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
