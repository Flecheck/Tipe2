use specs::{Component, Entity, System, VecStorage, WriteStorage};
use systems::propagation::Emission;
use specs::{Join, ParJoin};
use rayon::iter::ParallelIterator;
use TIME_PER_BEAT;

pub struct SimpleWaveEmitter {
    phase: f32,
    pulse: f32,
}

impl SimpleWaveEmitter {
    pub fn new(pulse: f32) -> Self {
        Self {
            phase: 0.0,
            pulse,
        }
    }
}

impl Component for SimpleWaveEmitter {
    type Storage = VecStorage<SimpleWaveEmitter>;
}

pub struct SimpleWave;

impl<'a> System<'a> for SimpleWave {
    type SystemData = (WriteStorage<'a, Emission>, WriteStorage<'a, SimpleWaveEmitter>);

    fn run(&mut self, (mut emission, mut state): Self::SystemData) {
        //println!("simple");
        (&mut emission, &mut state).join().for_each(|(emit, state)| {
            emit.current = state.phase.sin();
            state.phase = (state.phase + state.pulse * TIME_PER_BEAT) % (2.0 * std::f32::consts::PI);
        });
    }
}