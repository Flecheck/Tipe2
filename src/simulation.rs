use crossbeam_channel::{bounded, Receiver, Sender};
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::partitioning::BVT;
use serde::{Deserialize, Serialize};
use std::cmp::{min, max};
use std::thread;
use CHANNEL_BOUND;

use antennas::{SceneObject, SerializableWorld, SignalEmitter, SignalReceiver, WorldDescriptor};

use specs::{World, System};

struct Simulation {
    world: World,
}

impl Simulation {
    fn new() -> Self {
        Self {
            world: World::new(),
        }
    }

    fn start(&mut self) {

    }
}




