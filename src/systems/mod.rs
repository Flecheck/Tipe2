use ncollide3d::math::Isometry;
use specs::{Component, VecStorage};

pub mod propagation;
pub mod simple_wave;
pub mod tracker;

pub struct AntennaPosition {
    pub position: Isometry<f32>,
}

impl Component for AntennaPosition {
    type Storage = VecStorage<AntennaPosition>;
}