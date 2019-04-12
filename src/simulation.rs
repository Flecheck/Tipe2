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
    obstacles: Vec<StoredObstacle>,
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
    pub fn new() -> Self {
        Self {
            world: World::new(),
            obstacles: Vec::new(),
        }
    }

    pub fn save(&mut self, path: &str) {
        let mut entity_resolve: HashMap<Entity, u32> = HashMap::new();
        let mut count = 0;
        self.world.exec(
            |(entities, emission, reception): (
                Entities,
                ReadStorage<Emission>,
                ReadStorage<Reception>,
            )| {
                (&*entities, &emission, &reception)
                    .join()
                    .for_each(|(e, _, _)| {
                        entity_resolve.insert(e, count);
                        count += 1;
                    });
            },
        );

        let mut antennas: Vec<StoredAntenna> = (0..count).map(|_| Default::default()).collect();

        let reception_store = self.world.read_resource::<ReadStorage<Reception>>();
        let positions = self.world.read_resource::<ReadStorage<AntennaPosition>>();
        for _i in 0..count {
            for (e, k) in &entity_resolve {
                let recs = reception_store
                    .get(*e)
                    .expect("Unreachable: entity has no reception");
                let mut antenna = StoredAntenna {
                    position: positions
                        .get(*e)
                        .expect("Unreachable: antenna has no position")
                        .position,
                    transfer_matrix: Vec::new(),
                };
                for (target, data, _) in &recs.transfer {
                    antenna.transfer_matrix.push((
                        *entity_resolve
                            .get(target)
                            .expect("Unreachable: entity not a receiver"),
                        data.clone(),
                    ));
                }
                antennas[*k as usize] = antenna;
            }
        }

        let simulation = StoredSimulation {
            obstacles: self.obstacles.clone(),
            antennas,
        };

        let data = ron::ser::to_string_pretty(&simulation, ron_pretty())
            .expect("Failed to serialize the simulation");
        std::fs::write(path, data);
    }

    pub fn load(&mut self, path: &str) {
        let _sim: StoredSimulation = ron::de::from_reader(std::io::BufReader::new(
            std::fs::File::open(path).expect("Could not open simulation file"),
        ))
        .expect("Could not deserialize simulation");

        unimplemented!()
    }

    pub fn solve(&mut self, mut world: WorldDescriptor) {
        crate::waves::tracing(&mut world);

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

#[derive(Serialize, Deserialize, Clone)]
struct StoredSimulation {
    obstacles: Vec<StoredObstacle>,
    antennas: Vec<StoredAntenna>,
}

#[derive(Serialize, Deserialize, Clone)]
struct StoredObstacle {
    mesh_location: String,
    position: Isometry<f32>,
    n: f32,
}

#[derive(Serialize, Deserialize, Clone)]
struct StoredAntenna {
    position: Isometry<f32>,
    transfer_matrix: Vec<(u32, Vec<SignalEvent>)>,
}

impl Default for StoredAntenna {
    fn default() -> Self {
        Self {
            position: Isometry::new([0.0, 0.0, 0.0].into(), [0.0, 0.0, 0.0].into()),
            transfer_matrix: Vec::new(),
        }
    }
}

struct Name {
    name: String,
}

impl Component for Name {
    type Storage = VecStorage<Name>;
}
