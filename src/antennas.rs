use nalgebra::Point3;
use ncollide3d::shape::{ShapeHandle, Shape};
use ncollide3d::partitioning::BVT;
use ncollide3d::bounding_volume::aabb::AABB;

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
    pub collisions: BVT<ShapeHandle<f32>, AABB<f32>>,
}
