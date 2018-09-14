use antennas::{create_bvt_tuple, SignalEmitter, SignalReceiver, WorldDescriptor};
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

    let pos = Isometry3::new(Vector3::new(0.5, 0.5, 0.5), zero());
    let obstacle = Cuboid::new(Vector3::new(0.5, 0.5, 0.5));
    let collisions = BVT::new_balanced(vec![create_bvt_tuple(&obstacle, pos)]);

    WorldDescriptor {
        emitters,
        receivers,
        collisions,
    }
}
