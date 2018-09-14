#![feature(nll)]

extern crate crossbeam_channel;
extern crate nalgebra;
extern crate ncollide3d;
extern crate rayon;

mod antennas;
mod waves;

pub type Float = f32;

pub const WAVE_VELOCITY: Float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: Float = 60_000_000_000.;
pub const TIME_PER_BEAT: Float = 1. / MAX_FREQUENCY; // seconds

fn main() {
    println!("Hello, world!");
}
