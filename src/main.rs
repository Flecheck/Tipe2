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
extern crate itertools;

use clap::{App, SubCommand};

mod antennas;
mod simulation;
mod systems;
mod transfer;
mod waves;
mod world;
mod constants;

use antennas::create_bvt_tuple;

use ncollide3d::partitioning::BVT;
use ncollide3d::shape::Cuboid;
use ncollide3d::shape::Plane;
use nalgebra::Isometry3;
use nalgebra::Translation3;
use nalgebra::Point3;
use nalgebra::Unit;
use nalgebra::UnitQuaternion;

pub type Float = f32;

pub const WAVE_VELOCITY: Float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: Float = 60_000_000_000.;
pub const TIME_PER_BEAT: Float = 1. / MAX_FREQUENCY; // seconds

pub const CHANNEL_BOUND: usize = 65536;

fn main() {
    let mut sim = simulation::Simulation::new();

    let collisions = vec![
        create_bvt_tuple(
            &Plane::new(Unit::new_normalize([0.0, 1.0, 0.0].into())), 
            //&Cuboid::new([128.0, 128.0, 128.0].into()), 
            Isometry3::from_parts(Translation3::new(0.0, -8.0, 0.0), UnitQuaternion::identity()), 
            *constants::RefractiveIndices::soil,
        ),
        create_bvt_tuple(
            &Cuboid::new([2.0, 2.0, 2.0].into()), 
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity()), 
            1.0
        )];

    let description = antennas::WorldDescriptor {
        emitters: vec![None, Some(antennas::SignalEmitter {
            position: Point3::new(4.0, 0.0, 0.0),
            max_power: 1.0,
        }), Some(antennas::SignalEmitter {
            position: Point3::new(2.0, 2.0, 3.0),
            max_power: 2.0,
        })],
        receivers: vec![ Some(antennas::SignalReceiver {
            position: Point3::new(-4.0, 0.0, 0.0),
            transfers: vec![vec![], vec![]],
        }), None, None
        ],
        names: vec!["first".into(), "second".into(), "third".into()],
        collisions,
    };

    println!("Solving...");
    let time = chrono::Duration::span(|| sim.solve(description));
    println!("Solved in {} seconds", time.num_seconds());
    println!("Running...");
    let time = chrono::Duration::span(|| sim.start(vec!["first".into(), "second".into()]));
    println!("Ran in {} seconds", time.num_seconds());
    println!("Done my dudes!");
}
