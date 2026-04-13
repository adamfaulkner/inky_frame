use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rp_pico::hal::Timer;

use crate::inky73::DISPLAY_BUFFER_SIZE;

const DEBUG_SD_LOGGING: bool = false;

struct FakeTime {}

impl TimeSource for FakeTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp::from_fat(0, 0)
    }
}

pub struct InkySdCard<SPI: embedded_hal::spi::SpiDevice> {
    vmgr: VolumeManager<SdCard<SPI, Timer>, FakeTime>,
}

impl<SPI: embedded_hal::spi::SpiDevice> InkySdCard<SPI> {
    pub fn new(spi_device: SPI, d: Timer) -> Self {
        let sdcard = SdCard::new(spi_device, d);
        InkySdCard {
            vmgr: VolumeManager::new(sdcard, FakeTime {}),
        }
    }

    pub fn read_file_with_index(
        &mut self,
        index: usize,
        buf: &mut [u8; DISPLAY_BUFFER_SIZE],
    ) -> Result<usize, usize> {
        let mut volume = match self.vmgr.open_volume(VolumeIdx(0)) {
            Ok(x) => x,
            Err(_) => return Err(0),
        };

        let mut root_dir = volume.open_root_dir().unwrap();

        let mut primary_filename = [0u8; 12];
        let primary_name = filename_from_index(index, &mut primary_filename);
        let mut fallback_filename = [0u8; 12];
        let fallback_name = filename_from_index(0, &mut fallback_filename);

        let primary_read = {
            match root_dir.open_file_in_dir(primary_name, embedded_sdmmc::Mode::ReadOnly) {
                Ok(mut file) => {
                    file.seek_from_start(0u32).unwrap();
                    match file.read(buf) {
                        Ok(read_bytes) => Some(read_bytes),
                        Err(_) => return Err(4),
                    }
                }
                Err(_) => None,
            }
        };

        let (resolved_index, read_bytes) = match primary_read {
            Some(read_bytes) => (index, read_bytes),
            None => {
                let mut file = match root_dir
                    .open_file_in_dir(fallback_name, embedded_sdmmc::Mode::ReadOnly)
                {
                    Ok(file) => file,
                    Err(_) => return Err(3),
                };
                file.seek_from_start(0u32).unwrap();
                match file.read(buf) {
                    Ok(read_bytes) => (0, read_bytes),
                    Err(_) => return Err(4),
                }
            }
        };

        if read_bytes != buf.len() {
            // return Err(5);
        }

        if DEBUG_SD_LOGGING {
            let mut log_file = root_dir
                .open_file_in_dir("log", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)
                .unwrap();

            log_file.write(b"hello\n").unwrap();
            let logged_name = if resolved_index == index {
                primary_name
            } else {
                fallback_name
            };
            log_file.write(logged_name.as_bytes()).unwrap();
            log_file.write(b"\n").unwrap();
            log_file.write(&read_bytes.to_be_bytes()).unwrap();
            log_file.close().unwrap();
        }

        Ok(resolved_index)
    }
}

fn filename_from_index<'a>(index: usize, out: &'a mut [u8; 12]) -> &'a str {
    out.fill(b' ');

    let mut index_digits = [b'0'; 8];
    let mut index_digits_len = 0;
    if index == 0 {
        index_digits[0] = b'0';
        index_digits_len = 1;
    } else {
        let mut remaining = index;
        while remaining > 0 {
            index_digits[index_digits_len] = b'0' + (remaining % 10) as u8;
            remaining /= 10;
            index_digits_len += 1;
        }
    }

    let mut filename_len = 0;
    for digit in index_digits[..index_digits_len].iter().rev() {
        out[filename_len] = *digit;
        filename_len += 1;
    }
    out[filename_len] = b'.';
    filename_len += 1;
    out[filename_len] = b'I';
    out[filename_len + 1] = b'N';
    out[filename_len + 2] = b'K';
    filename_len += 3;

    core::str::from_utf8(&out[..filename_len]).unwrap()
}
