use std::{
    env,
    fs::{self, File},
    io::{Read, Write},
    path::Path,
};

use graphics::{convert_image, DISPLAY_BUFFER_SIZE, DISPLAY_HEIGHT, DISPLAY_WIDTH};
use image::{DynamicImage, ImageDecoder, ImageReader, RgbImage};
use libheif_rs::{ColorSpace, HeifContext, LibHeif, RgbChroma};

mod graphics;

fn is_heif_file(path: &Path) -> bool {
    let mut header = [0; 12];
    let Ok(mut file) = File::open(path) else {
        return false;
    };

    file.read_exact(&mut header).is_ok() && &header[4..8] == b"ftyp"
}

fn load_heif_image(path: &Path) -> DynamicImage {
    let context = HeifContext::read_from_file(path.to_str().unwrap()).unwrap();
    let handle = context.primary_image_handle().unwrap();
    let image = LibHeif::new()
        .decode(&handle, ColorSpace::Rgb(RgbChroma::Rgb), None)
        .unwrap();
    let plane = image.planes().interleaved.unwrap();
    let row_size = plane.width as usize * 3;
    let mut pixels = vec![0; row_size * plane.height as usize];

    for (src, dst) in plane
        .data
        .chunks_exact(plane.stride)
        .zip(pixels.chunks_exact_mut(row_size))
    {
        dst.copy_from_slice(&src[..row_size]);
    }

    DynamicImage::ImageRgb8(RgbImage::from_raw(plane.width, plane.height, pixels).unwrap())
}

fn load_image(path: &Path) -> DynamicImage {
    if is_heif_file(path) {
        return load_heif_image(path);
    }

    // Handle orientation correctly. Some file formats allow rotation without actually rotating
    // the image data.
    let mut decoder = ImageReader::open(path)
        .unwrap()
        .with_guessed_format()
        .unwrap()
        .into_decoder()
        .unwrap();
    let orientation = decoder.orientation().unwrap();

    let mut dynamic_image: DynamicImage = DynamicImage::from_decoder(decoder).unwrap();
    dynamic_image.apply_orientation(orientation);
    dynamic_image
}

/// Iterate over the input directory and convert each image to an InkyFrame format.
fn main() {
    let args: Vec<String> = env::args().collect();
    let input_file_base_path = &args[1];
    let output_file_base_path = &args[2];

    fs::create_dir_all(output_file_base_path).unwrap();

    for (idx, entry_result) in fs::read_dir(input_file_base_path).unwrap().enumerate() {
        let entry = entry_result.unwrap();
        let path = entry.path();
        if path.is_dir() {
            continue;
        }

        let dynamic_image = load_image(&path);

        // In my tests, I've found that increasing contrast can improve the quality of the image.
        let img: RgbImage = dynamic_image
            .thumbnail(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32)
            .adjust_contrast(30.0)
            .into();
        let mut output_buffer = [0u8; DISPLAY_BUFFER_SIZE];
        convert_image(&img, &mut output_buffer);

        let output_path = format!("{}/{}.INK", output_file_base_path, idx);

        let mut file = File::create(output_path).unwrap();
        file.write_all(&output_buffer).unwrap();
    }
    println!("done");
}
