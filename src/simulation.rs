use crossbeam_channel::{bounded, Receiver, Sender};
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::partitioning::BVT;
use serde::{Deserialize, Serialize};
use std::cmp::{min, max};
use std::thread;
use CHANNEL_BOUND;

use world::World;

use antennas::{SceneObject, SerializableWorld, SignalEmitter, SignalReceiver, WorldDescriptor};

pub type AntennaID = usize;

#[derive(Serialize, Deserialize)]
pub struct AntennaAllocator {
    next_antenna: AntennaID,
    emitters: Vec<SignalEmitter>,
    receivers: Vec<SignalReceiver>,
}

impl AntennaAllocator {
    pub fn new() -> Self {
        AntennaAllocator {
            next_antenna: 0,
            emitters: Vec::new(),
            receivers: Vec::new(),
        }
    }

    pub fn allocate(
        &mut self,
        emitter: Option<SignalEmitter>,
        receiver: Option<SignalReceiver>,
    ) -> AntennaID {
        let allocated = self.next_antenna;
        if let Some(emitter) = emitter {
            self.emitters[allocated] = emitter;
        }
        if let Some(receiver) = receiver {
            self.receivers[allocated] = receiver;
        }
        self.next_antenna += 1;
        allocated
    }

    pub fn len(&self) -> AntennaID {
        self.next_antenna
    }
}

pub struct AntennaStream {
    channel: Receiver<f32>,
}

impl AntennaStream {
    pub fn pop(&self, amount: usize) -> Vec<f32> {
        let mut amount = min(amount, self.channel.len());
        let mut res: Vec<f32> = Vec::with_capacity(amount);
        while amount > 0 {
            res.push(self.channel.recv().unwrap());
            amount -= 1;
        }
        res
    }
}

pub struct AntennaSink {
    channel: Sender<f32>,
}

impl AntennaSink {
    pub fn push(&self, data: Vec<f32>) {
        for x in data {
            self.channel.send(x);
        }
    }
}

pub trait System {
    fn register_antennas(&mut self, world: &World, antenna_allocator: &mut AntennaAllocator);
    fn run(&mut self, receivers: Vec<AntennaStream>, emitters: Vec<AntennaSink>);
}

pub struct Simulation {
    receivers_storage: Vec<AntennaStream>,
    emitters_storage: Vec<AntennaSink>,
    process_emitters: Vec<Receiver<f32>>,
    process_receivers: Vec<Sender<f32>>,

    transfer: Vec<Vec<(f32, f32)>>, // indexed by receiver, (time, gain)

    systems: Vec<Box<dyn System>>,
}

impl Simulation {
    pub fn with<T: 'static + System>(mut self, system: T) -> Self {
        self.systems.push(Box::new(system));
        self
    }

    pub fn solve(&mut self, world: World) -> WorldDescriptor {
        let mut allocator = AntennaAllocator::new();
        for s in &mut self.systems {
            s.register_antennas(&world, &mut allocator);
        }

        let total_allocated = allocator.len();

        /* Build the world */
        let mut world = WorldDescriptor {
            emitters: allocator.emitters,
            receivers: allocator.receivers,
            collisions: world.get_bvt(),
        };

        use waves::tracing;
        tracing(&mut world);

        world
    }

    pub fn run(&mut self, emitters: Vec<SignalEmitter>, receivers: Vec<SignalReceiver>) {
        let total_allocated = max(emitters.len(), receivers.len());
        for k in 0..total_allocated {
            let (send, recv) = bounded(CHANNEL_BOUND);
            self.receivers_storage[k] = AntennaStream { channel: recv };
            self.process_receivers[k] = send;
            let (send, recv) = bounded(CHANNEL_BOUND);
            self.emitters_storage[k] = AntennaSink { channel: send };
            self.process_emitters[k] = recv;
        }


    }
}
