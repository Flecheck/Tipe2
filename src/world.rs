use antennas::{create_bvt_tuple, SignalEmitter, SignalReceiver, WorldDescriptor};
use nalgebra::{zero, Isometry3, Point3, Vector3};
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::bounding_volume::HasBoundingVolume;
use ncollide3d::partitioning::BVT;
use ncollide3d::query::RayCast;
use ncollide3d::shape::Cuboid;

use antennas::SceneObject;

pub struct Singularity {
    label: String,
    position: Point3<f32>,
}

pub struct World {
    singularities: Vec<Singularity>,
    objects: Vec<(SceneObject, AABB<f32>)>,
}

impl World {
    pub fn insert_object<
        T: 'static + Send + Sync + Clone + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
    >(
        &mut self,
        object: T,
        position: Isometry3<f32>,
    ) {
        self.objects.push(create_bvt_tuple(&object, position, 1.));
    }

    pub fn insert(&mut self, singularity: Singularity) {
        self.singularities.push(singularity);
    }

    pub fn get_bvt(self) -> BVT<SceneObject, AABB<f32>> {
        BVT::new_balanced(self.objects)
    }
}

pub fn dummy_world() -> WorldDescriptor {
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
    let collisions = BVT::new_balanced(vec![create_bvt_tuple(&obstacle, pos, 1.)]);

    WorldDescriptor {
        emitters,
        receivers,
        collisions,
    }
}
