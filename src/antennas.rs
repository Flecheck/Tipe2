use nalgebra::Point3;
use ncollide3d::shape::ShapeHandle;

pub struct SignalReceiver {
    pub position: Point3<f32>,
}

pub struct SignalEmitter {
    pub position: Point3<f32>,
    pub max_power: f32,

    pub transfers: Vec<Vec<(u32, f32)>>, // indexed by emitter, (time, gain)
}

pub struct WorldDescriptor {
    pub emitters: Vec<SignalEmitter>,
    pub receivers: Vec<SignalReceiver>,
    pub collisions: Vec<ShapeHandle<f32>>,
}
