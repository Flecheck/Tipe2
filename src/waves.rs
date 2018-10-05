use antennas::{WorldDescriptor, SignalEvent};
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use ncollide3d::query::RayIntersectionCostFn;
use ncollide3d::shape::ShapeHandle;
use rayon;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use std;
use WAVE_VELOCITY;

use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::partitioning::BVT;

use antennas::ClosestRayTOICostFn;
use antennas::{SceneObject, SignalEmitter, SignalReceiver, WorldDescriptor};
use ncollide3d::query::RayIntersection;

use nalgebra::{dot, norm, normalize};

const NB_SAMPLE: u32 = 100;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;
const MIN_ENERGY: f32 = 0.001;
const LOST_PER_BOUNCE: f32 = 0.7;

/// (Ray,power,distance,max_energy)
type EnergyRay = (Ray<f32>, f32, f32, f32);

/// Do the ray tracing and populate emetters with receivers
pub fn tracing(world: &mut WorldDescriptor) {
    let (so, ro) = channel::bounded(10_000);

    let rays = world
        .emitters
        .iter()
        .enumerate()
        .flat_map(|(ide, x)| {
            emit(&x.position)
                .map(move |ray| (ray, x.max_power, 0.))
                .map(move |ray| (ide, ray))
        }).collect::<Vec<_>>();

    rayon::scope(|s| {
        let ref receivers = world.receivers;
        let ref collisions = world.collisions;
        s.spawn(move |s| {
            rays.into_iter()
                .map(|(ide, (ray, power, dist))| (ide, (ray, power, dist, power)))
                .for_each(|x| process(x, &so, receivers, collisions));
        });

        for (idr, receiver) in receivers.iter().enumerate() {
            for emitter in world.emitters.iter_mut() {
                if line(&receiver, emitter.position, &collisions) {
                    let dist = norm(&(receiver.position - emitter.position));
                    let power = emitter.max_power;
                    emitter.transfers[idr].push(((dist / WAVE_VELOCITY).floor() as u32, power));
                }
            }
        }

        for (ide, idr, time, power) in ro {
            world.emitters[ide].transfers[idr].push(SignalEvent { time, gain: power });
        }
    })
}

fn process(
    (ide, (ray, pow, dist, max_energy)): (usize, EnergyRay),
    out: &channel::Sender<(usize, usize, u32, f32)>,
    receivers: &Vec<SignalReceiver>,
    bvs: &BVT<SceneObject, AABB<f32>>,
) {
    if pow < MIN_ENERGY {
        return;
    }
    let mut visitor = ClosestRayTOICostFn::new(&ray);
    if let Some(inter) = bvs.best_first_search(&mut visitor) {
        let dist_plus = norm(&(ray.dir * inter.1.toi));

        find_receiver(&ray, pow, &inter, receivers, bvs)
            .into_iter()
            .map(|(idr, energy, dist)| (ide, idr, energy, dist + dist_plus))
            .filter(|r| r.2 < MIN_ENERGY)
            .map(|(ide, idr, energy, dist)| {
                (ide, idr, (dist / WAVE_VELOCITY).floor() as u32, energy)
            }).for_each(|c| out.send(c));

        let pow = pow * LOST_PER_BOUNCE;

        next_rays(&ray, &inter)
            .into_par_iter()
            .map(|ray| (ray, pow, dist + dist_plus, max_energy))
            .map(|x| (ide, x))
            .for_each(|x| process(x, out, receivers, bvs));
    }
}

fn emit<'a>(pos: &'a Point3<f32>) -> impl Iterator<Item = Ray<f32>> + 'a {
    (0..NB_SAMPLE).flat_map(move |alpha| {
        (0..NB_SAMPLE).map(move |beta| {
            let x = (alpha as f32 * 2f32 * PI / NB_SAMPLEF).cos();
            let y = (alpha as f32 * 2f32 * PI / NB_SAMPLEF).sin();
            let z = (beta as f32 * 2f32 * PI / NB_SAMPLEF).sin();
            Ray::new(pos.clone(), Vector3::new(x, y, z))
        })
    })
}

// Lamberian Reflectance
fn find_receiver(
    ray: &Ray<f32>,
    power: f32,
    inter: &(&SceneObject, RayIntersection<f32>),
    receivers: &Vec<SignalReceiver>,
    bvs: &BVT<SceneObject, AABB<f32>>,
) -> Vec<(usize, f32, f32)> {
    let normal = normalize(&inter.1.normal);
    let pos = ray.origin + ray.dir * inter.1.toi;

    let receivers = receivers
        .into_iter()
        .enumerate()
        .filter(|(i, x)| line(x, pos, bvs));

    receivers
        .map(|(i, receiver)| {
            let vec = receiver.position - pos;
            let nvec = normalize(&vec);
            let power_percentage = dot(&nvec, &normal);
            let dist = norm(&vec);
            (i, power * power_percentage, dist)
        }).collect()
}

fn line(receiver: &SignalReceiver, point: Point3<f32>, bvs: &BVT<SceneObject, AABB<f32>>) -> bool {
    let ray = Ray::new(point, receiver.position - point);
    let mut visitor = ClosestRayTOICostFn::new(&ray);
    if let Some(inter) = bvs.best_first_search(&mut visitor) {
        if norm(&(ray.origin - point)) > norm(&(ray.dir * inter.1.toi)) {
            return false;
        }
    }
    true
}

fn next_rays(ray: &Ray<f32>, inter: &(&SceneObject, RayIntersection<f32>)) -> Vec<Ray<f32>> {
    let normal = normalize(&inter.1.normal);

    let reflection = 2. * dot(&normal, &ray.dir) * normal - ray.dir;

    vec![Ray::new(ray.origin + ray.dir * inter.1.toi, reflection)]
}
