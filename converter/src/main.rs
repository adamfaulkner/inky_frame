use std::{env, fs::File, io::Write};

use graphics::{convert_image, DISPLAY_BUFFER_SIZE, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use image::{ImageReader, RgbImage};

mod graphics;

fn main() {
    let args: Vec<String> = env::args().collect();
    let input_filename = &args[1];
    let output_filename = &args[2];
    let img: RgbImage = ImageReader::open(input_filename)
        .unwrap()
        .decode()
        .unwrap()
        .thumbnail(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32)
        .into();
    let mut output_buffer = [0u8; DISPLAY_BUFFER_SIZE];
    convert_image(&img, &mut output_buffer);

    let mut file = File::create(output_filename).unwrap();
    file.write_all(&output_buffer).unwrap();
    println!("done");
}
