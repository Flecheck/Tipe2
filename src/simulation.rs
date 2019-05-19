use crossbeam_channel::{bounded, Receiver, Sender};
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::math::Isometry;
use ncollide3d::partitioning::BVT;
use serde::{Deserialize, Serialize};
use std::cmp::{max, min};
use std::collections::{HashMap, VecDeque};
use std::thread;
use crate::CHANNEL_BOUND;

use crate::antennas::{SceneObject, SerializableWorld, SignalEvent, SignalReceiver, WorldDescriptor};
use crate::systems::AntennaPosition;
use crate::systems::{
    propagation::{Emission, PropagationSystem, Reception},
    simple_wave::{SimpleWave, SimpleWaveEmitter},
    ofdm::{OFDMEmitter, OFDMReceiver},
    tracker::TrackerSystem,
};

use specs::{
    Component, DispatcherBuilder, Entities, Entity, Join, ReadStorage, VecStorage, World,
    WriteStorage,
};

use ron::ser::PrettyConfig;

fn ron_pretty() -> PrettyConfig {
    PrettyConfig {
        new_line: "\n".to_owned(),
        indentor: "\t".to_owned(),
        ..Default::default()
    }
}

fn check_valid(transf: &Vec<SignalEvent>) -> bool {
    for i in 1..transf.len() {
        if transf[i - 1].time >= transf[i].time {
            return false;
        }
    }
    true
}

pub struct Simulation {
    pub world: World,
    pub descriptor: WorldDescriptor,
}

#[derive(PartialEq, Eq)]
pub enum AntennaKind {
    Emit,
    Rece,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum EmissionKind {
    Pulse(f32),
    OFDM(Vec<u8>),
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ReceptionKind {
    None,
    OFDM,
}

impl Simulation {
    pub fn new(descriptor: WorldDescriptor) -> Self {
        Self {
            world: World::new(),
            descriptor,
        }
    }

    pub fn save_solution(&mut self, path: &str) {
        let descriptor = &self.descriptor;

        let serializable = crate::antennas::SerializableWorld {
            emitters: descriptor.emitters.clone(),
            receivers: descriptor.receivers.clone(),
            names: descriptor.names.clone(),
        };

        let data = ron::ser::to_string_pretty(&serializable, ron_pretty())
            .expect("Failed to serialize the simulation");
        std::fs::write(path, data);
    }

    pub fn from_solution(path: &str) -> Self {
        let serializable: crate::antennas::SerializableWorld = ron::de::from_reader(std::io::BufReader::new(
            std::fs::File::open(path).expect("Could not open simulation file"),
        ))
        .expect("Could not deserialize simulation");

        Self {
            world: World::new(),
            descriptor: WorldDescriptor {
                emitters: serializable.emitters,
                receivers: serializable.receivers,
                names: serializable.names,
                collisions: Vec::new(),
            }
        }
    }

    pub fn solve(&mut self) {
        crate::waves::tracing(&mut self.descriptor);
    }

    pub fn instanciate(&mut self) {
        let world = &self.descriptor;

        self.world.register::<Reception>();
        self.world.register::<Emission>();
        self.world.register::<OFDMReceiver>();
        self.world.register::<OFDMEmitter>();
        self.world.register::<Name>();
        self.world.register::<SimpleWaveEmitter>();

        let mut entities: Vec<(AntennaKind, Entity)> = Vec::with_capacity(world.names.len());
        for i in 0..world.names.len() {
            use specs::Builder;
            if let Some(ref emit) = world.emitters[i] {
                let antenna = self
                    .world
                    .create_entity()
                    .with(Emission { current: 0.0, label: world.names[i].clone() });
                let antenna = match emit.kind {
                    EmissionKind::Pulse(pulse) => antenna.with(SimpleWaveEmitter::new(pulse)),
                    EmissionKind::OFDM(ref data) => antenna.with(OFDMEmitter::new(data)),
                };
                let antenna = antenna.build();
                entities.push((AntennaKind::Emit, antenna));
            } else if let Some(ref rec) = world.receivers[i] {
                let antenna = self
                    .world
                    .create_entity();
                let antenna = match rec.kind {
                    ReceptionKind::None => antenna,
                    ReceptionKind::OFDM => antenna.with(OFDMReceiver::new()),
                };
                entities.push((AntennaKind::Rece, antenna.build()));
            }
        }

        self.world.exec(|mut recs: WriteStorage<Reception>| {
            for i in 0..world.names.len() {
                if let Some(ref rec) = world.receivers[i] {
                    let mut transfer = Vec::with_capacity(rec.transfers.len());
                    for k in 0..rec.transfers.len() {
                        if entities[k].0 == AntennaKind::Emit {
                            if !check_valid(&rec.transfers[k]) {
                                panic!("Transfer function for rec {} emit {} not sorted", i, k);
                            }

                            // (entity, transf, max_time)
                            transfer.push((
                                entities[k].1,
                                rec.transfers[k].clone(),
                                rec.transfers[k]
                                    .iter()
                                    .map(|x| x.time)
                                    .max()
                                    .map(|x| x + 1)
                                    .unwrap_or(0),
                            ));
                        }
                    }
                    recs.insert(
                        entities[i].1,
                        Reception::new(transfer, world.names[i].clone()),
                    )
                    .expect("Unreachable: failed to insert Reception");
                }
            }
        });
    }

    pub fn start(&mut self, names: Vec<String>, time: usize) {
        let mut dispatcher = DispatcherBuilder::new()
            .with(SimpleWave, "simple_wave", &[])
            .with(crate::systems::ofdm::OFDMEmit, "odfm_emit", &[])
            .with(crate::systems::ofdm::OFDMReceive, "ofdm_rec", &[])
            .with(PropagationSystem, "propagation_system", &[])
            .with(TrackerSystem::new(names), "tracker_system", &[])
            .build();

        println!("Dispatching!");

        for _ in 0..time {
            dispatcher.dispatch(&mut self.world.res);
        }
    }
}

struct Name {
    name: String,
}

impl Component for Name {
    type Storage = VecStorage<Name>;
}
