use super::propagation::Reception;
use specs::{Component, Entity, Join, ReadStorage, System, VecStorage, WriteStorage};

pub struct ProxyReception {
    pub current: f32,
    pub label: String,
}

impl ProxyReception {
    pub fn new(label: String) -> Self {
        Self {
            current: 0.0,
            label,
        }
    }
}

impl Component for ProxyReception {
    type Storage = VecStorage<ProxyReception>;
}

pub struct MovementHandler {
    current_pos: usize,
    antenna_cycle: Vec<Entity>,
}

impl MovementHandler {
    pub fn new(antenna_cycle: Vec<Entity>) -> Self {
        Self {
            current_pos: 0,
            antenna_cycle,
        }
    }
}

impl<'a> System<'a> for MovementHandler {
    type SystemData = (ReadStorage<'a, Reception>, WriteStorage<'a, ProxyReception>);

    fn run(&mut self, (reception, mut proxy): Self::SystemData) {
        (&mut proxy).join().for_each(|x| {
            x.current = reception
                .get(
                    *self
                        .antenna_cycle
                        .get(self.current_pos)
                        .expect("Movement antenna out of bounds"),
                )
                .expect("Antenna entity no longer alive")
                .current;
        });
        self.current_pos += 1;
        if self.current_pos >= self.antenna_cycle.len() {
            self.current_pos = 0;
        }
    }
}