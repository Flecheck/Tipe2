#![allow(dead_code)]

extern crate clap;
extern crate crossbeam_channel;
extern crate crossbeam_deque;
extern crate nalgebra;
extern crate ncollide3d;
extern crate rayon;
extern crate rustfft;
extern crate serde;
#[macro_use]
extern crate serde_derive;
#[macro_use]
extern crate lazy_static;
extern crate bit_vec;
extern crate chrono;
extern crate itertools;

mod antennas;
mod constants;
mod ring_buffer;
mod simulation;
mod systems;
mod waves;
mod world;

use nalgebra::Point3;

use specs::ReadStorage;

pub type Float = f32;

pub const WAVE_VELOCITY: Float = 299_792_458.; // meters per second
pub const MAX_FREQUENCY: Float = 60_000_000_000.;
pub const TIME_PER_BEAT: Float = 1. / MAX_FREQUENCY; // seconds

pub const CHANNEL_BOUND: usize = 65536;

fn main() {
    let collisions = world::basic_collisions();//absurd_collisions(500, [128.0, 128.0, 128.0]);

    // Battements
    /*let description = antennas::WorldDescriptor {
        emitters: vec![
            None,
            Some(antennas::SignalEmitter {
                position: Point3::new(6.0, 0.0, 0.0),
                max_power: 1.0,
                kind: simulation::EmissionKind::Pulse(1000000000.0),
            }),
            Some(antennas::SignalEmitter {
                position: Point3::new(-8.0, 0.0, 0.0),
                max_power: 1.0,
                kind: simulation::EmissionKind::Pulse(1100000000.0),
            }),
        ],
        receivers: vec![
            Some(antennas::SignalReceiver {
                position: Point3::new(0.0, 0.0, 0.0),
                transfers: vec![vec![], vec![], vec![]],
                kind: simulation::ReceptionKind::None,
            }),
            None,
            None,
        ],
        names: vec!["first".into(), "second".into(), "third".into()],
        collisions,
    };*/

    // OFDM
    
    let description = antennas::WorldDescriptor {
        emitters: vec![
            None,
            Some(antennas::SignalEmitter {
                position: Point3::new(-5.0, 0.0, 0.0),
                max_power: 10.0,
                kind: simulation::EmissionKind::OFDM(vec![0xBE, 0xEF, 0xCA, 0xFE]),
            }),
        ],
        receivers: vec![
            Some(antennas::SignalReceiver {
                position: Point3::new(5.0, 0.0, 0.0),
                transfers: vec![vec![], vec![]],
                kind: simulation::ReceptionKind::OFDM,
            }),
            None,
        ],
        names: vec!["ofdm_rec".into(), "ofdm_emit".into()],
        collisions,
    };

    let mut sim = simulation::Simulation::new(description);

    println!("Solving...");
    let time = chrono::Duration::span(|| sim.solve());
    println!("Solved in {} seconds", time.num_seconds());
    //println!("Saving solution...");
    //sim.save_solution("output/solution.ron");
    //let mut sim = simulation::Simulation::from_solution("output/solution.ron");
    println!("Instanciating solution...");
    sim.instanciate();
    println!("Running...");
    let time =
        chrono::Duration::span(|| sim.start(vec!["ofdm_rec".into(), "ofdm_emit".into()], 0x20000));
    println!("Ran in {} seconds", time.num_seconds());
    println!("Gathering OFDM results...");
    sim.world.exec(
        |(recs, or): (
            ReadStorage<systems::propagation::Reception>,
            ReadStorage<systems::ofdm::OFDMReceiver>,
        )| {
            use specs::Join;
            (&recs, &or).join().for_each(|(x, r)| {
                println!("{}: {:?}", x.label, r.data_buffer);
            });
        },
    );
    println!("Done my dudes!");
}
