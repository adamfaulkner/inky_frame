use core::{
    mem::swap,
    ops::{Add, AddAssign, Div, Mul},
};

static PALETTE: [Rgb888; 8] = [
    Rgb888::new(0, 0, 0),
    Rgb888::new(255, 255, 255),
    Rgb888::new(0, 255, 0),
    Rgb888::new(0, 0, 255),
    Rgb888::new(255, 0, 0),
    Rgb888::new(255, 255, 0),
    Rgb888::new(255, 128, 0),
    Rgb888::new(220, 180, 200),
];

#[derive(Copy, Clone)]
pub struct PixelError {
    r: i16,
    g: i16,
    b: i16,
}
impl PixelError {
    fn default() -> PixelError {
        PixelError { r: 0, g: 0, b: 0 }
    }
}

impl Mul<i16> for PixelError {
    type Output = Self;

    fn mul(self, rhs: i16) -> Self::Output {
        PixelError {
            r: self.r * rhs,
            g: self.g * rhs,
            b: self.b * rhs,
        }
    }
}

// Note: I bet this compiles badly when it could just be a >> for 16
impl Div<i16> for PixelError {
    type Output = Self;

    fn div(self, rhs: i16) -> Self::Output {
        PixelError {
            r: self.r / rhs,
            g: self.g / rhs,
            b: self.b / rhs,
        }
    }
}

impl Add for PixelError {
    type Output = Self;
    fn add(self, rhs: PixelError) -> Self::Output {
        PixelError {
            r: (self.r + rhs.r).min(255).max(-255),
            g: (self.g + rhs.g).min(255).max(-255),
            b: (self.b + rhs.b).min(255).max(-255),
        }
    }
}

impl AddAssign for PixelError {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

fn error_between_colors(from: Rgb888, to: Rgb888) -> PixelError {
    PixelError {
        r: ((from.r() as i16) - (to.r() as i16)).min(255).max(-255),
        g: ((from.g() as i16) - (to.g() as i16)).min(255).max(-255),
        b: ((from.b() as i16) - (to.b() as i16)).min(255).max(-255),
    }
}

fn add_error(from: Rgb888, err: PixelError) -> Rgb888 {
    Rgb888::new(
        (from.r() as i16 + err.r).max(0).min(255) as u8,
        (from.g() as i16 + err.g).max(0).min(255) as u8,
        (from.b() as i16 + err.b).max(0).min(255) as u8,
    )
}

#[derive(Clone, Copy)]
enum InkyColor {
    Black,
    White,
    Green,
    Blue,
    Red,
    Yellow,
    Orange,
    Clean,
}

impl InkyColor {
    fn to_color_number(self) -> u8 {
        match self {
            InkyColor::Black => 0,
            InkyColor::White => 1,
            InkyColor::Green => 2,
            InkyColor::Blue => 3,
            InkyColor::Red => 4,
            InkyColor::Yellow => 5,
            InkyColor::Orange => 6,
            InkyColor::Clean => 7,
        }
    }
    fn from_color_number(i: usize) -> Self {
        match i {
            0 => InkyColor::Black,
            1 => InkyColor::White,
            2 => InkyColor::Green,
            3 => InkyColor::Blue,
            4 => InkyColor::Red,
            5 => InkyColor::Yellow,
            6 => InkyColor::Orange,
            7 => InkyColor::Clean,
            _ => panic!("Invalid color number given to from_color_number"),
        }
    }

    fn to_rgb888(self) -> Rgb888 {
        PALETTE[self.to_color_number() as usize]
    }

    fn error_from(self, from: Rgb888) -> PixelError {
        error_between_colors(from, self.to_rgb888())
    }
}

fn color_distance_sq(in1: Rgb888, in2: Rgb888) -> i32 {
    ((in1.r() as i32 - in2.r() as i32) as i32).pow(2)
        + ((in1.g() as i32 - in2.g() as i32) as i32).pow(2)
        + ((in1.b() as i32 - in2.b() as i32) as i32).pow(2)
}

fn closest_color(input: Rgb888) -> (InkyColor, PixelError) {
    let color_idx = PALETTE
        .iter()
        .enumerate()
        .min_by_key(|(_, color)| color_distance_sq(input, **color))
        .unwrap()
        .0;

    let color = InkyColor::from_color_number(color_idx);
    (color, color.error_from(input))
}

fn inky_colors_to_output(in1: InkyColor, in2: InkyColor) -> u8 {
    (in1.to_color_number() << 4) | (in2.to_color_number())
}

fn convert_single_pixel(
    i: usize,
    j: usize,
    bmp: &Bmp<Rgb888>,
    current_row_error: &mut [PixelError; DISPLAY_WIDTH],
    next_row_error: &mut [PixelError; DISPLAY_WIDTH],
    next_next_row_error: &mut [PixelError; DISPLAY_WIDTH],
) -> InkyColor {
    let pixel = bmp
        .pixel(Point {
            y: i as i32,
            x: j as i32,
        })
        .unwrap();

    let accumulated_error = current_row_error[j];
    let (inky_color, new_error) = closest_color(add_error(pixel, accumulated_error));
    //let (inky_color, new_error) = closest_color(pixel);

    // Propagate the new error forward.

    // Floyd-Steinberg
    /*
    let new_error_div = new_error / 16;
    current_row_error[j + 1] += new_error_div * 7;
    if j > 0 {
        next_row_error[j - 1] += new_error_div * 3;
    }
    next_row_error[j] += new_error_div * 5;
    if j < DISPLAY_WIDTH - 1 {
        next_row_error[j + 1] += new_error_div;
    }
    */

    // Atkinson
    let new_error_div = new_error / 8;
    next_row_error[j] += new_error_div;
    next_next_row_error[j] += new_error_div;
    if j < DISPLAY_WIDTH - 1 {
        current_row_error[j + 1] += new_error_div;
        next_row_error[j + 1] += new_error_div;
    }
    if j < DISPLAY_WIDTH - 2 {
        current_row_error[j + 2] += new_error_div;
    }
    if j > 0 {
        next_row_error[j - 1] += new_error_div;
    }

    inky_color
}

pub fn convert_image<'a, SPI: embedded_hal::spi::SpiDevice>(
    sd_card: InkySdCard<'a, SPI>,
    output_buffer: &mut [u8; DISPLAY_BUFFER_SIZE],
) {
    let mut read_buf: [u8; 4096] = [0; 4096];
    let reader_adapter = SdCardReaderAdapter::new(sd_card);
    let qoi_decoder = QoiDecoder::new(reader_adapter);

    let read_bytes = sd_card.read_image(&mut read_buf);

    let mut row_a_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut row_b_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut row_c_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut current_row_error = &mut row_a_error;
    let mut next_row_error = &mut row_b_error;
    let mut next_next_row_error = &mut row_c_error;

    for i in 0..qoi_header.height() {
        // Swap the error rows and reset the next_row_error.
        if i > 0 {
            swap(&mut current_row_error, &mut next_row_error);
            swap(&mut next_row_error, &mut next_next_row_error);
            for i in next_next_row_error.iter_mut() {
                (*i) = PixelError::default();
            }
        }

        // Note that j ranges over indexes in the display, rather than pixels in the source image.
        // Since each byte of display holds two pixels, we range over this / 2
        for j in 0..(qoi_header.width() / 2) {
            let inky_color_1 = convert_single_pixel(
                i as usize,
                (j * 2) as usize,
                &bmp,
                current_row_error,
                next_row_error,
                next_next_row_error,
            );
            let inky_color_2 = convert_single_pixel(
                i as usize,
                ((j * 2) + 1) as usize,
                &bmp,
                current_row_error,
                next_row_error,
                next_next_row_error,
            );

            let output = inky_colors_to_output(inky_color_1, inky_color_2);
            output_buffer[(i * (DISPLAY_WIDTH as u32 / 2) + j) as usize] = output;
        }
    }
}
