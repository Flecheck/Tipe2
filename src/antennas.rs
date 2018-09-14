use nalgebra::Point3;
use ncollide3d::shape::ShapeHandle;

pub struct SignalReceiver {
    position: Point3<f32>,
    max_power: f32,
}

pub struct SignalEmitter {
    position: Point3<f32>,
    max_power: f32,

    transfers: Vec<Vec<(u32, f32)>>, // indexed by emitter, (time, gain)
}

pub struct WorldDescriptor {
    emitters: Vec<SignalEmitter>,
    receivers: Vec<SignalReceiver>,
    collisions: Vec<ShapeHandle<f32>>,
}