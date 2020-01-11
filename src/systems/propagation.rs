use crate::antennas::SignalEvent;
use crate::ring_buffer::RingBuffer;
use specs::{Component, Entity, ReadStorage, System, VecStorage, WriteStorage};

pub struct Emission {
    pub current: f32,
    pub label: String,
}

impl Component for Emission {
    type Storage = VecStorage<Emission>;
}

pub struct Reception {
    pub current: f32,
    receive_buffer: RingBuffer<f32>,
    pub transfer: Vec<(Entity, Vec<SignalEvent>, usize)>,
    pub label: String,
}

impl Reception {
    pub fn new(transfer: Vec<(Entity, Vec<SignalEvent>, usize)>, name: impl ToString) -> Self {
        Self {
            current: 0.0,
            receive_buffer: RingBuffer::with_capacity(
                transfer
                    .iter()
                    .map(|(_, _, x)| x)
                    .max()
                    .expect("Could not find max of max_size")
                    + 1,
            ),
            transfer,
            label: name.to_string(),
        }
    }
}

impl Component for Reception {
    type Storage = VecStorage<Reception>;
}

struct UnsafePointer(*mut RingBuffer<f32>);

unsafe impl Send for UnsafePointer {}
unsafe impl Sync for UnsafePointer {}

pub struct PropagationSystem;

impl<'a> System<'a> for PropagationSystem {
    type SystemData = (ReadStorage<'a, Emission>, WriteStorage<'a, Reception>);

    fn run(&mut self, (emission, mut reception): Self::SystemData) {
        use rayon::prelude::ParallelIterator;
        use specs::ParJoin;
        (&mut reception,).par_join().for_each(|(rec,)| {
            for (entity, events, max_time) in rec.transfer.iter() {
                if let Some(emit) = emission.get(*entity) {
                    // Parallel version
                    /*
                    use rayon::iter::IntoParallelIterator;
                    let ptr = UnsafePointer(&mut rec.receive_buffer);
                    unsafe {
                        events.into_iter().for_each(|e| {
                            *(*ptr.0).get_mut(e.time).unwrap_or_else(|| {
                                panic!(
                                    "Unreachable: sample not allocated, max_time: {}, capacity: {}",
                                    max_time,
                                    rec.receive_buffer.len(),
                                )
                            }) += emit.current * e.gain;
                        });
                    }*/

                    // Sequential version
                    for e in events.into_iter() {
                        *rec.receive_buffer.get_mut(e.time).unwrap_or_else(|| {
                            panic!("Unreachable: sample not allocated, max_time: {}", max_time)
                        }) += emit.current * e.gain;
                    }
                }
            }

            rec.current = rec.receive_buffer.pop();
        });
    }
}
