use crate::systems::propagation::Emission;
use crate::TIME_PER_BEAT;
use specs::Join;
use specs::{Component, System, VecStorage, WriteStorage};

pub struct SimpleWaveEmitter {
    phase: f32,
    pulse: f32,
}

impl SimpleWaveEmitter {
    pub fn new(pulse: f32) -> Self {
        Self { phase: 0.0, pulse }
    }
}

impl Component for SimpleWaveEmitter {
    type Storage = VecStorage<SimpleWaveEmitter>;
}

pub struct SimpleWave;

impl<'a> System<'a> for SimpleWave {
    type SystemData = (
        WriteStorage<'a, Emission>,
        WriteStorage<'a, SimpleWaveEmitter>,
    );

    fn run(&mut self, (mut emission, mut state): Self::SystemData) {
        //println!("simple");
        (&mut emission, &mut state)
            .join()
            .for_each(|(emit, state)| {
                emit.current = state.phase.sin();
                state.phase =
                    (state.phase + state.pulse * TIME_PER_BEAT) % (2.0 * std::f32::consts::PI);
            });
    }
}
