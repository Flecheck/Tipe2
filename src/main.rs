extern crate clap;
extern crate crossbeam_channel;
extern crate crossbeam_deque;
extern crate nalgebra;
extern crate ncollide3d;
extern crate rayon;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate rand;
extern crate ron;
extern crate specs;

use clap::{App, SubCommand};

mod antennas;
mod constants;
mod simulation;
mod systems;
mod transfer;
mod waves;
mod world;

pub type Float = f32;

pub const WAVE_VELOCITY: Float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: Float = 60_000_000_000.;
pub const TIME_PER_BEAT: Float = 1. / MAX_FREQUENCY; // seconds

pub const CHANNEL_BOUND: usize = 65536;

fn main() {
    let matches = App::new("Tipe2")
        .version("1.0")
        .author(
            "Théo Degioanni <moxinilian@tutanota.com> & Mathieu Bessou <bessou.mathieu@gmail.com>",
        )
        .subcommand(SubCommand::with_name("transfer").about("Builds transfer functions"))
        .subcommand(SubCommand::with_name("simulate").about("Runs the simulation"))
        .subcommand(SubCommand::with_name("view").about("Opens the 3D view"))
        .get_matches();

    if let Some(_) = matches.subcommand_matches("transfer") {
        transfer::transfer();
    } else {
        println!("Unimplemented yet.\nUse 'help' for help.");
    }
}
