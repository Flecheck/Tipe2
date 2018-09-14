#![feature(nll)]

extern crate nalgebra;
extern crate ncollide3d;

mod antennas;

pub type float = f32;

pub const WAVE_VELOCITY: float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: float = 60_000_000_000.;
pub const TIME_PER_BEAT: float = 1. / MAX_FREQUENCY; // seconds

fn main() {
    println!("Hello, world!");
}
