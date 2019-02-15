use antennas::SignalEvent;
use specs::{Component, Entity, ReadStorage, System, VecStorage, WriteStorage};
use std::collections::VecDeque;

pub struct Emission {
    pub current: f32,
    //samples: VecDeque<f32>,
    pub transfer: Vec<(Entity, Vec<SignalEvent>, usize)>,
}

impl Component for Emission {
    type Storage = VecStorage<Emission>;
}

pub struct Reception {
    pub current: f32,
    receive_buffer: VecDeque<f32>,
}

impl Reception {
    pub fn new() -> Self {
        Self {
            current: 0.0,
            receive_buffer: VecDeque::new(),
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
        use specs::Join;
        for (emit,) in (&emission,).join() {
            for (entity, events, max_time) in emit.transfer.iter() {
                if let Some(rec) = reception.get_mut(*entity) {
                    if rec.receive_buffer.len() < *max_time {
                        // Potentially faster to do it in one iteration
                        rec.receive_buffer.resize(*max_time, 0.0);
                    }
                    for e in events.into_iter() {
                        *rec.receive_buffer
                            .get_mut(e.time)
                            .expect("Unreachable: sample not allocated") += emit.current * e.gain;
                    }
                }
            }
        }

        use rayon::prelude::{IndexedParallelIterator, ParallelIterator};
        (&mut reception,).join().for_each(|(rec,)| {
            rec.current = rec
                .receive_buffer
                .pop_front()
                .expect("Unreachable: receive_buffer empty");
        });
    }
}
