use embedded_hal::i2c::{I2c, Operation, SevenBitAddress};
use rp_pico::hal::i2c::Error;

// Lots of this code copied from
// https://github.com/tweedegolf/pcf85063a/blob/main/src/lib.rs#L80
// I didn't want to bother with this dependency since it was async.

const DEVICE_ADDRESS: u8 = 0b1010001;

pub struct Pcf85063a<'a> {
    i2c: &'a mut dyn I2c<SevenBitAddress, Error = Error>,
}

impl<'a> Pcf85063a<'a> {
    pub fn new(i2c: &'a mut dyn I2c<SevenBitAddress, Error = Error>) -> Self {
        Self { i2c }
    }

    pub fn get_ram_byte(&mut self) -> u8 {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(DEVICE_ADDRESS, &[0x03], &mut data)
            .unwrap();
        data[0]
    }

    pub fn put_ram_byte(&mut self, data: u8) -> () {
        self.i2c.write(DEVICE_ADDRESS, &[0x03, data]).unwrap()
    }

    pub fn set_seconds_timer(&mut self, seconds: u8) -> () {
        self.i2c
            .transaction(
                DEVICE_ADDRESS,
                &mut [
                    // Set timer value to the input number of seconds.
                    Operation::Write(&[0x10, seconds]),
                    // Set timer control to 1hz, timer enable, timer interrupt enabled, timer interrupt mode = flag
                    Operation::Write(&[0x11, 0b00010110]),
                ],
            )
            .unwrap();
    }

    pub fn set_minutes_timer(&mut self, minutes: u8) -> () {
        // This is exactly the same as `set_seconds_timer` except that we use a slower clock.
        self.i2c
            .transaction(
                DEVICE_ADDRESS,
                &mut [
                    // Set timer value to the input number of minutes.
                    Operation::Write(&[0x10, minutes]),
                    // Set timer control to 1/60hz, timer enable, timer interrupt enabled, timer interrupt mode = flag
                    Operation::Write(&[0x11, 0b00011110]),
                ],
            )
            .unwrap();
    }

    pub fn clear_timer_flag(&mut self) -> () {
        self.i2c
            .transaction(
                DEVICE_ADDRESS,
                &mut [
                    // Disable the timer.
                    Operation::Write(&[0x11, 0b00010010]),
                    // Clear the flag. note that this might wipe the alarm?
                    Operation::Write(&[0x01, 0b00000000]),
                ],
            )
            .unwrap()
    }
}
