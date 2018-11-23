use crossbeam_channel::{bounded, Receiver, Sender};
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::math::Isometry;
use ncollide3d::partitioning::BVT;
use serde::{Deserialize, Serialize};
use std::cmp::{max, min};
use std::collections::HashMap;
use std::thread;
use CHANNEL_BOUND;

use antennas::{SceneObject, SerializableWorld, SignalEvent, SignalReceiver, WorldDescriptor};
use systems::propagation::{Emission, PropagationSystem, Reception};
use systems::AntennaPosition;

use specs::{DispatcherBuilder, Entities, Entity, Join, ReadStorage, World};

use ron::ser::PrettyConfig;

fn ron_pretty() -> PrettyConfig {
    PrettyConfig {
        new_line: "\n".to_owned(),
        indentor: "\t".to_owned(),
        ..Default::default()
    }
}

struct Simulation {
    world: World,
    obstacles: Vec<StoredObstacle>,
}

impl Simulation {
    fn new() -> Self {
        Self {
            world: World::new(),
            obstacles: Vec::new(),
        }
    }

    fn save(&mut self, path: &str) {
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

        let emission_store = self.world.read_resource::<ReadStorage<Emission>>();
        let positions = self.world.read_resource::<ReadStorage<AntennaPosition>>();
        for i in 0..count {
            for (e, k) in &entity_resolve {
                let emitter = emission_store
                    .get(*e)
                    .expect("Unreachable: entity has no emitter");
                let mut antenna = StoredAntenna {
                    position: positions
                        .get(*e)
                        .expect("Unreachable: antenna has no position")
                        .position,
                    transfer_matrix: Vec::new(),
                };
                for (target, data, _) in &emitter.transfer {
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

    fn load(&mut self) {
        unimplemented!()
    }

    fn solve(&mut self) {
        unimplemented!()
    }

    fn start(&mut self) {
        let mut dispatcher = DispatcherBuilder::new()
            .with(PropagationSystem, "propagation_system", &[])
            .build();

        loop {
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