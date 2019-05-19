use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::bounding_volume::HasBoundingVolume;
use ncollide3d::query::RayCast;

use antennas::create_bvt_tuple;
use antennas::SceneObject;

use constants;

use nalgebra::Isometry3;
use nalgebra::Point3;
use nalgebra::Translation3;
use nalgebra::Unit;
use nalgebra::UnitQuaternion;
use ncollide3d::partitioning::BVT;
use ncollide3d::shape::Cuboid;
use ncollide3d::shape::Plane;

fn plane(pos: [f32; 3], n: f32, normal: [f32; 3]) -> (SceneObject, AABB<f32>) {
    create_bvt_tuple(
        &Plane::new(Unit::new_normalize(normal.into())),
        Isometry3::from_parts(
            Translation3::new(pos[0], pos[1], pos[2]),
            UnitQuaternion::identity(),
        ),
        n,
    )
}

fn cuboid(pos: [f32; 3], n: f32, half_diag: [f32; 3]) -> (SceneObject, AABB<f32>) {
    create_bvt_tuple(
        &Cuboid::new(half_diag.into()),
        Isometry3::from_parts(
            Translation3::new(pos[0], pos[1], pos[2]),
            UnitQuaternion::identity(),
        ),
        n,
    )
}

pub fn basic_collisions() -> Vec<(SceneObject, AABB<f32>)> {
    vec![
        /*plane(
            [0.0, -8.0, 0.0],
            *constants::RefractiveIndices::soil,
            [0.0, 1.0, 0.0],
        ),*/
        cuboid([0.0; 3], 1.0, [2.0; 3]),
    ]
}

pub fn complex_collisions() -> Vec<(SceneObject, AABB<f32>)> {
    vec![
        plane( // Ground
            [0.0, -8.0, 0.0],
            *constants::RefractiveIndices::soil,
            [0.0, 1.0, 0.0],
        ),
        cuboid([0.0; 3], 1.0, [2.0; 3]),
        cuboid([5.0, 0.0, 5.0], 1.3, [2.0; 3]),
        cuboid([-5.0, 0.0, 5.0], 1.4, [2.0; 3]),
        cuboid([-12.0, 0.0, 0.0], 2.0, [1.0; 3]),
        cuboid([12.0, 3.0, 0.0], 2.0, [1.0; 3]),
        //cuboid()
    ]
}