use antennas::SignalEvent;
use specs::{Component, Entity, ReadStorage, System, VecStorage, WriteStorage};
use std::collections::VecDeque;

pub struct Emission {
    pub current: f32,
}

impl Component for Emission {
    type Storage = VecStorage<Emission>;
}

pub struct Reception {
    pub current: f32,
    receive_buffer: VecDeque<f32>,
    pub transfer: Vec<(Entity, Vec<SignalEvent>, usize)>,
    pub label: String,
}

impl Reception {
    pub fn new(transfer: Vec<(Entity, Vec<SignalEvent>, usize)>, name: impl ToString) -> Self {
        Self {
            current: 0.0,
            receive_buffer: VecDeque::new(),
            transfer,
            label: name.to_string(),
        }
    }
}

impl Component for Reception {
    type Storage = VecStorage<Reception>;
}

pub struct PropagationSystem;

impl<'a> System<'a> for PropagationSystem {
    type SystemData = (ReadStorage<'a, Emission>, WriteStorage<'a, Reception>);

    fn run(&mut self, (emission, mut reception): Self::SystemData) {
        //println!("propa");
        use specs::ParJoin;
        use rayon::prelude::ParallelIterator;
        (&mut reception,).par_join().for_each(|(rec,)| {
            for (entity, events, max_time) in rec.transfer.iter() {
                if let Some(emit) = emission.get(*entity) {
                    if rec.receive_buffer.len() < *max_time {
                        // Potentially faster to do it in one iteration
                        rec.receive_buffer.resize(*max_time, 0.0);
                    }
                    
                    for e in events.into_iter() {
                        *rec.receive_buffer
                            .get_mut(e.time)
                            .unwrap_or_else(|| panic!("Unreachable: sample not allocated, max_time: {}", max_time)) += emit.current * e.gain;
                    }
                }
            }

            //println!("{:?}", rec.receive_buffer);
            rec.current = rec
                .receive_buffer
                .pop_front()
                .expect("Isolated antenna");
        });
    }
}
