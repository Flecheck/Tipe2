use nalgebra::Point3;
use ncollide3d::bounding_volume::aabb;
use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::bounding_volume::HasBoundingVolume;
use ncollide3d::math::Isometry;
use ncollide3d::partitioning::BVTCostFn;
use ncollide3d::partitioning::BVT;
use ncollide3d::query::Ray;
use ncollide3d::query::RayCast;
use ncollide3d::query::RayIntersection;

pub struct SignalReceiver {
    pub position: Point3<f32>,
}

pub struct SignalEvent {
    pub time: u32,
    pub gain: f32,
}

pub struct SignalEmitter {
    pub position: Point3<f32>,
    pub max_power: f32,

    pub transfers: Vec<Vec<SignalEvent>>, // indexed by receiver, (time, gain)
}

pub struct WorldDescriptor {
    pub emitters: Vec<SignalEmitter>,
    pub receivers: Vec<SignalReceiver>,
    pub collisions: BVT<SceneObject, AABB<f32>>,
}

pub struct SceneObject {
    geometry: Box<RayCast<f32> + Sync + Send>,
    transform: Isometry<f32>,
}
impl SceneObject {
    pub fn new<G>(geometry: Box<G>, transform: Isometry<f32>) -> SceneObject
    where
        G: 'static + Sync + Send + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
    {
        SceneObject {
            geometry,
            transform,
        }
    }
    pub fn cast(&self, ray: &Ray<f32>) -> Option<RayIntersection<f32>> {
        self.geometry
            .toi_and_normal_with_ray(&self.transform, ray, false)
    }
}

pub struct ClosestRayTOICostFn<'a> {
    ray: &'a Ray<f32>,
}
impl<'a> ClosestRayTOICostFn<'a> {
    pub fn new(ray: &'a Ray<f32>) -> ClosestRayTOICostFn<'a> {
        ClosestRayTOICostFn { ray }
    }
}
impl<'a> BVTCostFn<f32, SceneObject, AABB<f32>> for ClosestRayTOICostFn<'a> {
    type UserData = RayIntersection<f32>;
    fn compute_bv_cost(&mut self, bv: &AABB<f32>) -> Option<f32> {
        bv.toi_with_ray(&Isometry::identity(), self.ray, true)
    }
    fn compute_b_cost(&mut self, b: &SceneObject) -> Option<(f32, RayIntersection<f32>)> {
        b.cast(self.ray).map(|inter| (inter.toi, inter))
    }
}

pub fn create_bvt_tuple<G>(shape: &G, transform: Isometry<f32>) -> (SceneObject, AABB<f32>)
where
    G: 'static + Send + Sync + Clone + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
{
    (
        SceneObject::new(Box::new(shape.clone()), transform),
        aabb(shape, &transform),
    )
}
