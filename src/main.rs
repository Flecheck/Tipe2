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
extern crate byteorder;
#[macro_use]
extern crate lazy_static;
extern crate chrono;

use clap::{App, SubCommand};

mod antennas;
mod simulation;
mod systems;
mod transfer;
mod waves;
mod world;

use ncollide3d::partitioning::BVT;
use nalgebra::Point3;

pub type Float = f32;

pub const WAVE_VELOCITY: Float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: Float = 60_000_000_000.;
pub const TIME_PER_BEAT: Float = 1. / MAX_FREQUENCY; // seconds

pub const CHANNEL_BOUND: usize = 65536;

fn main() {
    let mut sim = simulation::Simulation::new();

    let description = antennas::WorldDescriptor {
        emitters: vec![None, Some(antennas::SignalEmitter {
            position: Point3::new(4.0, 0.0, 0.0),
            max_power: 1.0,
        })],
        receivers: vec![ Some(antennas::SignalReceiver {
            position: Point3::new(-4.0, 0.0, 0.0),
            transfers: vec![vec![], vec![]],
        }), None
        ],
        names: vec!["first".into(), "second".into()],
        collisions: vec![],
    };

    println!("Solving...");
    let time = chrono::Duration::span(|| sim.solve(description));
    println!("Solved in {} seconds", time.num_seconds());
    println!("Running...");
    let time = chrono::Duration::span(|| sim.start(vec!["first".into(), "second".into()]));
    println!("Ran in {} seconds", time.num_seconds());
    println!("Done my dudes!");
}
