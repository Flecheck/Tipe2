use ncollide3d::math::Isometry;
use specs::{Component, VecStorage};

pub mod propagation;
pub mod simple_wave;
pub mod ofdm;
pub mod tracker;
pub mod moving;

pub struct AntennaPosition {
    pub position: Isometry<f32>,
}

impl Component for AntennaPosition {
    type Storage = VecStorage<AntennaPosition>;
}
