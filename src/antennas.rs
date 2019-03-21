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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalReceiver {
    pub position: Point3<f32>,
    pub transfers: Vec<Vec<SignalEvent>>, // indexed by emitter, (time, gain)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalEvent {
    pub time: usize,
    pub gain: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalEmitter {
    pub position: Point3<f32>,
    pub max_power: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SerializableWorld {
    pub emitters: Vec<SignalEmitter>,
    pub receivers: Vec<SignalReceiver>,
}

pub struct WorldDescriptor {
    pub emitters: Vec<SignalEmitter>,
    pub receivers: Vec<SignalReceiver>,
    pub names: Vec<String>,
    pub collisions: Vec<(SceneObject, AABB<f32>)>,
}   

pub struct SceneObject {
    geometry: Box<RayCast<f32> + Sync + Send>,
    transform: Isometry<f32>,
    pub n: f32,
    pub receiver: Option<usize>,
}

impl SceneObject {
    pub fn new<G>(geometry: Box<G>, transform: Isometry<f32>, n: f32, receiver: Option<usize>) -> SceneObject
    where
        G: 'static + Sync + Send + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
    {
        SceneObject {
            geometry,
            transform,
            n,
            receiver: receiver,
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

pub fn create_bvt_tuple<G>(shape: &G, transform: Isometry<f32>, n: f32) -> (SceneObject, AABB<f32>)
where
    G: 'static + Send + Sync + Clone + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
{
    (
        SceneObject::new(Box::new(shape.clone()), transform, n,None),
        aabb(shape, &transform),
    )
}

pub fn create_bvt_tuple_receiver<G>(shape: &G, transform: Isometry<f32>,receiver: usize) -> (SceneObject, AABB<f32>)
where
    G: 'static + Send + Sync + Clone + RayCast<f32> + HasBoundingVolume<f32, AABB<f32>>,
{
    (
        SceneObject::new(Box::new(shape.clone()), transform, 1.,Some(receiver)),
        aabb(shape, &transform),
    )
}
