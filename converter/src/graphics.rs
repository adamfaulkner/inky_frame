/// Contains logic to convert an input image into the 8 color format supported by the inky frame.
use core::{
    mem::swap,
    ops::{Add, AddAssign, Div, Mul},
};

use image::{GenericImageView, Rgb};

// Inky frame's display has this many pixels.
pub const DISPLAY_WIDTH: usize = 800;
pub const DISPLAY_HEIGHT: usize = 480;
// However, the inky frame represents two pixels with one byte, so it has half the buffer size.
pub const DISPLAY_BUFFER_SIZE: usize = 800 * 480 / 2;

/// InkyColor represents the different colors supported by the inky frame.
#[derive(Clone, Copy, PartialEq, Eq)]
struct InkyColor(Rgb<u8>);

/// These are the 8 colors supported by the inky frame, in order.
const BLACK: InkyColor = InkyColor(image::Rgb([0, 0, 0]));
const WHITE: InkyColor = InkyColor(image::Rgb([255, 255, 255]));
const GREEN: InkyColor = InkyColor(image::Rgb([0, 255, 0]));
const BLUE: InkyColor = InkyColor(image::Rgb([0, 0, 255]));
const RED: InkyColor = InkyColor(image::Rgb([255, 0, 0]));
const YELLOW: InkyColor = InkyColor(image::Rgb([255, 255, 0]));
const ORANGE: InkyColor = InkyColor(image::Rgb([255, 128, 0]));
const CLEAN: InkyColor = InkyColor(image::Rgb([220, 180, 200]));

/// The palette of colors supported by the inky frame, in order.
static PALETTE: [InkyColor; 8] = [BLACK, WHITE, GREEN, BLUE, RED, YELLOW, ORANGE, CLEAN];

/// PixelError tracks an accumulated difference between requested colors and the closest color in
/// the palette during the dithering operation..
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

fn error_between_colors(from: Rgb<u8>, to: Rgb<u8>) -> PixelError {
    PixelError {
        r: ((from[0] as i16) - (to[0] as i16)).min(255).max(-255),
        g: ((from[1] as i16) - (to[1] as i16)).min(255).max(-255),
        b: ((from[2] as i16) - (to[2] as i16)).min(255).max(-255),
    }
}

fn add_error(from: Rgb<u8>, err: PixelError) -> Rgb<u8> {
    Rgb::from([
        (from[0] as i16 + err.r).max(0).min(255) as u8,
        (from[1] as i16 + err.g).max(0).min(255) as u8,
        (from[2] as i16 + err.b).max(0).min(255) as u8,
    ])
}

impl InkyColor {
    fn to_color_number(self) -> u8 {
        match self {
            BLACK => 0,
            WHITE => 1,
            GREEN => 2,
            BLUE => 3,
            RED => 4,
            YELLOW => 5,
            ORANGE => 6,
            CLEAN => 7,
            _ => panic!("Invalid color number given to to_color_number"),
        }
    }
    fn from_color_number(i: usize) -> Self {
        match i {
            0 => BLACK,
            1 => WHITE,
            2 => GREEN,
            3 => BLUE,
            4 => RED,
            5 => YELLOW,
            6 => ORANGE,
            7 => CLEAN,
            _ => panic!("Invalid color number given to from_color_number"),
        }
    }

    fn to_rgb(self) -> Rgb<u8> {
        self.0
    }

    fn error_from(self, from: Rgb<u8>) -> PixelError {
        error_between_colors(from, self.to_rgb())
    }
}

fn color_distance_sq(in1: Rgb<u8>, in2: Rgb<u8>) -> i32 {
    ((in1[0] as i32 - in2[0] as i32) as i32).pow(2)
        + ((in1[1] as i32 - in2[1] as i32) as i32).pow(2)
        + ((in1[2] as i32 - in2[2] as i32) as i32).pow(2)
}

fn closest_color(input: Rgb<u8>) -> (InkyColor, PixelError) {
    let color_idx = PALETTE
        .iter()
        // CLEAN has some weird behavior when the frame is on battery power. On USB power, CLEAN
        // appears as expected, but on battery power, it is greenish, with an increasing green tint
        // towards the bottom of the screen. To avoid this, we just exclude it from the palette.
        .filter(|color| *color != &CLEAN)
        .enumerate()
        .min_by_key(|(_, color)| color_distance_sq(input, color.0))
        .unwrap()
        .0;

    let color = InkyColor::from_color_number(color_idx);
    (color, color.error_from(input))
}

fn inky_colors_to_output(in1: InkyColor, in2: InkyColor) -> u8 {
    (in1.to_color_number() << 4) | (in2.to_color_number())
}

/// Convert a single pixel to an InkyColor, propagating the error forward.
fn convert_single_pixel(
    i: usize,
    j: usize,
    image: &dyn GenericImageView<Pixel = Rgb<u8>>,
    current_row_error: &mut [PixelError; DISPLAY_WIDTH],
    next_row_error: &mut [PixelError; DISPLAY_WIDTH],
    _next_next_row_error: &mut [PixelError; DISPLAY_WIDTH],
) -> InkyColor {
    let pixel = image.get_pixel(j as u32, i as u32);

    let accumulated_error = current_row_error[j];
    let (inky_color, new_error) = closest_color(add_error(pixel, accumulated_error));

    // Propagate the new error forward.

    // Floyd-Steinberg dithering
    let new_error_div = new_error / 16;
    if j > 0 {
        next_row_error[j - 1] += new_error_div * 3;
    }
    next_row_error[j] += new_error_div * 5;
    if j < DISPLAY_WIDTH - 1 {
        current_row_error[j + 1] += new_error_div * 7;
        next_row_error[j + 1] += new_error_div;
    }

    // Atkinson - this is another option, sometimes it looks better.
    /*
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
    */

    inky_color
}

pub fn convert_image(
    image: &dyn GenericImageView<Pixel = Rgb<u8>>,
    output_buffer: &mut [u8; DISPLAY_BUFFER_SIZE],
) {
    let left_border = (DISPLAY_WIDTH as u32 - image.width()) / 2;
    let right_border = (image.width()) + left_border;
    let top_border = (DISPLAY_HEIGHT as u32 - image.height()) / 2;
    let bottom_border = (image.height()) + top_border;

    let mut row_a_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut row_b_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut row_c_error: [PixelError; DISPLAY_WIDTH] = [PixelError::default(); DISPLAY_WIDTH];
    let mut current_row_error = &mut row_a_error;
    let mut next_row_error = &mut row_b_error;
    let mut next_next_row_error = &mut row_c_error;

    for i in 0..(DISPLAY_HEIGHT as u32) {
        // Swap the error rows and reset the next_row_error.
        if i > 0 {
            swap(&mut current_row_error, &mut next_row_error);
            swap(&mut next_row_error, &mut next_next_row_error);
            for i in next_next_row_error.iter_mut() {
                (*i) = PixelError::default();
            }
        }

        // If i is above or below its top margin, then use WHITE
        if i < top_border || i >= bottom_border {
            for j in 0..(DISPLAY_WIDTH as u32 / 2) {
                output_buffer[(i * (DISPLAY_WIDTH as u32 / 2) + j) as usize] =
                    inky_colors_to_output(WHITE, WHITE);
            }
            continue;
        }

        // Note that j ranges over indexes in the display, rather than pixels in the source image.
        // Since each byte of display holds two pixels, we range over this / 2
        // TODO: clean up crappy code

        for j in 0..((DISPLAY_WIDTH / 2) as u32) {
            let inky_color_1 = if (j * 2) < left_border || (j * 2) >= right_border {
                WHITE
            } else {
                convert_single_pixel(
                    (i - top_border) as usize,
                    ((j * 2) - left_border) as usize,
                    image,
                    current_row_error,
                    next_row_error,
                    next_next_row_error,
                )
            };
            let inky_color_2 = if (j * 2 + 1) < left_border || (j * 2 + 1) >= right_border {
                WHITE
            } else {
                convert_single_pixel(
                    (i - top_border) as usize,
                    (((j * 2) + 1) - left_border) as usize,
                    image,
                    current_row_error,
                    next_row_error,
                    next_next_row_error,
                )
            };

            let output = inky_colors_to_output(inky_color_1, inky_color_2);
            output_buffer[(i * (DISPLAY_WIDTH as u32 / 2) + j) as usize] = output;
        }
    }
}
