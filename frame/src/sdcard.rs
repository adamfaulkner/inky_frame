use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rp_pico::hal::Timer;

use crate::inky73::DISPLAY_BUFFER_SIZE;

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

    #[allow(dead_code)]
    pub fn get_len(&mut self) -> u32 {
        let mut volume = self.vmgr.open_volume(VolumeIdx(0)).unwrap();

        let mut root_dir = volume.open_root_dir().unwrap();
        let file = root_dir
            .open_file_in_dir("b.out", embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        file.length()
    }
}
