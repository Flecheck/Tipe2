use antennas::{SignalEmitter, SignalReceiver, WorldDescriptor};
use nalgebra::{zero, Isometry3, Point3, Vector3};
use ncollide3d::partitioning::BVT;
use ncollide3d::shape::{Cuboid, ShapeHandle};

pub fn get_main_world() -> WorldDescriptor {
    let emitters = vec![SignalEmitter {
        position: Point3::new(-2.0, 0.0, 0.0),
        max_power: 16.0,
        transfers: vec![Vec::new()],
    }];
    let receivers = vec![SignalReceiver {
        position: Point3::new(2.0, 0.0, 0.0),
    }];

    let obstacle = ShapeHandle::new(Cuboid::new(Vector3::new(0.5, 0.5, 0.5)));
    let aabb = obstacle.aabb(&Isometry3::new(Vector3::new(0.5, 0.5, 0.5), zero()));
    let collisions = BVT::new_balanced(vec![(obstacle, aabb)]);

    WorldDescriptor {
        emitters,
        receivers,
        collisions,
    }
}
