use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::bounding_volume::HasBoundingVolume;
use ncollide3d::query::RayCast;

use crate::antennas::create_bvt_tuple;
use crate::antennas::SceneObject;

use crate::constants;

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
        cuboid([0.0; 3], 1.5, [2.0; 3]),
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

pub fn absurd_collisions(obstacles: u32, half_diag: [f32; 3]) -> Vec<(SceneObject, AABB<f32>)> {
    let mut res = vec![plane(
            [0.0, -half_diag[1] * 1.1, 0.0],
            *constants::RefractiveIndices::soil,
            [0.0, 1.0, 0.0],
        )];
    
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    // The biggest cube must be small enough so we can fit "obstacles" obstacles in the volume
    let max_size = ((half_diag[0] * half_diag[1] * half_diag[2]) / (obstacles as f32)).cbrt() * 0.5;

    for _ in 0..obstacles {
        let x = rng.gen::<f32>() * 2.0 * half_diag[0] - half_diag[0];
        let y = rng.gen::<f32>() * 2.0 * half_diag[1] - half_diag[1];
        let z = rng.gen::<f32>() * 2.0 * half_diag[2] - half_diag[2];

        let hw = rng.gen::<f32>() * max_size;
        let hh = rng.gen::<f32>() * max_size;
        let hd = rng.gen::<f32>() * max_size;

        res.push(cuboid([x, y, z], rng.gen::<f32>() + 0.5, [hw, hh, hd]));
    }

    res
}