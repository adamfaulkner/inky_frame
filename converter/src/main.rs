use std::{
    env,
    fs::{self, File},
    io::Write,
};

use graphics::{convert_image, DISPLAY_BUFFER_SIZE, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use image::{DynamicImage, ImageDecoder, ImageReader, RgbImage};

mod graphics;

fn main() {
    let args: Vec<String> = env::args().collect();
    let input_file_base_path = &args[1];
    let output_file_base_path = &args[2];

    for (idx, entry_result) in fs::read_dir(input_file_base_path).unwrap().enumerate() {
        let entry = entry_result.unwrap();
        let path = entry.path();
        if path.is_dir() {
            continue;
        }

        // Handle orientation correctly
        let mut decoder = ImageReader::open(path).unwrap().into_decoder().unwrap();
        let orientation = decoder.orientation().unwrap();

        let mut dynamic_image: DynamicImage = DynamicImage::from_decoder(decoder).unwrap();
        dynamic_image.apply_orientation(orientation);

        let img: RgbImage = dynamic_image
            .thumbnail(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32)
            .adjust_contrast(30.0)
            .into();
        let mut output_buffer = [0u8; DISPLAY_BUFFER_SIZE];
        convert_image(&img, &mut output_buffer);

        let output_path = format!("{}/{}.inkyframe", output_file_base_path, idx);

        let mut file = File::create(output_path).unwrap();
        file.write_all(&output_buffer).unwrap();
    }
    println!("done");
}
