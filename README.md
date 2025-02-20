# Inky Frame Photo Album in Rust

I made this project to learn about programming a raspberry pi pico in Rust, and to learn about how
to interact with hardware at a low level. I'm not sure this will be useful to anyone, but it was
fun.

Lots of this code was modified from Pimoroni's example code:
https://github.com/pimoroni/pimoroni-pico/tree/main/drivers/inky73

## Features

- Displays images loaded from an SD card
- `E` button advances image, `A` button shows previous image.
- Automatically advances image after 30 minutes.
- Powers itself off after changing image to conserve battery.
- "Flag" led shows when the frame is busy.
- Uses Floyd-Steinberg dithering to convert images to the format used by the frame.

## Usage

There are two programs here: `converter`, which converts images to the format used by the frame and
runs on a normal computer, and `frame`, which runs on the frame itself and displays images loaded
from the SD card.

### `converter`

`converter` takes two arguments: the path to a directory containing the images to convert, and the
path to the output directory. After running `converter`, copy the contents of the output directory to an SD card formatted with a FAT32 filesystem.

### `frame`

To load `frame` onto the the frame, plug the device in to the USB port and put it into bootloader
mode, then run `cargo run --release` to build and flash the firmware.
