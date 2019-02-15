#![allow(dead_code)]
use antennas::{SignalEvent, WorldDescriptor};
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
use antennas::{SceneObject, SignalEmitter, SignalReceiver};
use ncollide3d::query::RayIntersection;

use nalgebra::{dot, norm, normalize};

const NB_SAMPLE: u32 = 100;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;
const MIN_GAIN: f32 = 0.001;
const LOST_PER_BOUNCE: f32 = 0.7;
const N_AIR: f32 = 1.;

/// (Ray,energy,distance,max_energy,n)
struct EnergyRay {
    ray: Ray<f32>,
    energy: f32,
    distance: f32,
    max_energy: f32,
    n: f32,
}

struct Output {
    ide: usize,
    idr: usize,
    time: usize,
    energy: f32,
}

/// Do the ray tracing and populate emetters with receivers
pub fn tracing(world: &mut WorldDescriptor) {
    let (so, ro) = channel::bounded(10_000);

    // Starting rays
    let rays = world
        .emitters
        .iter()
        .enumerate()
        .flat_map(|(ide, x)| {
            emit(&x.position)
                .map(move |ray| (ray.0, x.max_power * ray.1, 0.))
                .map(move |ray| (ide, ray))
        })
        .collect::<Vec<_>>();

    rayon::scope(|s| {
        let ref receivers = world.receivers;
        let ref collisions = world.collisions;

        // Prosessing
        s.spawn(move |s| {
            rays.into_iter()
                .map(|(ide, (ray, energy, distance))| {
                    (
                        ide,
                        EnergyRay {
                            ray,
                            energy,
                            distance,
                            max_energy: energy,
                            n: 1.,
                        },
                    )
                })
                .for_each(|x| process(x, &so, receivers, collisions));
        });

        // Direct reach
        /* for (idr, receiver) in receivers.iter().enumerate() {
            for emitter in world.emitters.iter_mut() {
                if line(&receiver, emitter.position, &collisions) {
                    let dist = norm(&(receiver.position - emitter.position));
                    emitter.transfers[idr].push(SignalEvent {
                        time: (dist / WAVE_VELOCITY).floor() as usize,
                        gain: 1.,
                    });
                }
            }
        } */

        // Collecting
        for out in ro {
            world.emitters[out.ide].transfers[out.idr].push(SignalEvent {
                time: out.time,
                gain: out.energy,
            });
        }
    })
}

// TODO: Dephasage refraction
fn process(
    (ide, energyray): (usize, EnergyRay),
    out: &channel::Sender<Output>,
    receivers: &Vec<SignalReceiver>,
    bvs: &BVT<SceneObject, AABB<f32>>,
) {
    if energyray.energy / energyray.max_energy < MIN_GAIN {
        return;
    }
    let mut visitor = ClosestRayTOICostFn::new(&energyray.ray);
    if let Some(inter) = bvs.best_first_search(&mut visitor) {
        let dist_plus = norm(&(energyray.ray.dir * inter.1.toi)) / energyray.n;
        let mut n2 = inter.0.n;
        if n2 == energyray.n {
            n2 = N_AIR;
        }
        let n1 = energyray.n;

        let normal = normalize(&inter.1.normal);

        let cos1 = norm(&(dot(&normal, &energyray.ray.dir) * normal - energyray.ray.dir));
        let cos2 = (1. - (n1 / n2) * (n1 / n2) * (1. - cos1 * cos1).sqrt()).sqrt();

        let rtm = (n1 * cos2 - n2 * cos1) / (n1 * cos2 + n2 * cos1);
        let ttm = 1. - rtm;

        let mut nextrays = Vec::new();

        // Receivers
        find_receiver(&energyray.ray, energyray.energy, &inter, receivers, bvs)
            .into_iter()
            .map(|(idr, energy, dist)| (ide, idr, energy * rtm, dist + dist_plus))
            .filter(|r| r.2 / energyray.max_energy < MIN_GAIN) // Only reacheble antenas
            .map(|(ide, idr, energy, dist)| Output {
                ide,
                idr,
                time: (dist / WAVE_VELOCITY).floor() as usize,
                energy,
            })
            .for_each(|c| out.send(c));

        // Reflection
        {
            let reflection = next_rays_reflection(&energyray.ray, &inter);

            nextrays.push(EnergyRay {
                ray: reflection.translate_by(reflection.dir * 0.001),
                energy: energyray.energy * rtm,
                distance: energyray.distance + dist_plus,
                max_energy: energyray.max_energy,
                n: n1,
            });
        }

        // Refraction
        if n2 / n1 <= 1. {
            let refraction = next_rays_refraction(&energyray.ray, &inter, energyray.n, n2);
            nextrays.push(EnergyRay {
                ray: refraction.0.translate_by(refraction.0.dir * 0.001),
                energy: energyray.energy * ttm,
                distance: energyray.distance + dist_plus,
                max_energy: energyray.max_energy,
                n: n2,
            });
        }

        nextrays
            .into_par_iter()
            .map(|x| (ide, x))
            .for_each(|x| process(x, out, receivers, bvs));
    }
}

// TODO: Check every ray is normalized
fn emit<'a>(pos: &'a Point3<f32>) -> impl Iterator<Item = (Ray<f32>, f32)> + 'a {
    (0..NB_SAMPLE).flat_map(move |alpha| {
        (0..NB_SAMPLE).map(move |beta| {
            let phi = alpha as f32 * 2f32 * PI / NB_SAMPLEF;
            let theta = beta as f32 * 2f32 * PI / NB_SAMPLEF;
            let x = theta.sin() * phi.cos();
            let y = theta.sin() * phi.sin();
            let z = theta.sin();
            (
                Ray::new(pos.clone(), Vector3::new(x, y, z)),
                2. * phi * (1. - (theta / 2.).cos()),
            )
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
        })
        .collect()
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

fn next_rays_reflection(ray: &Ray<f32>, inter: &(&SceneObject, RayIntersection<f32>)) -> Ray<f32> {
    let normal = normalize(&inter.1.normal);

    let reflection = 2. * dot(&normal, &ray.dir) * normal - ray.dir;

    Ray::new(ray.origin + ray.dir * inter.1.toi, reflection)
}

fn next_rays_refraction(
    ray: &Ray<f32>,
    inter: &(&SceneObject, RayIntersection<f32>),
    n1: f32,
    n2: f32,
) -> (Ray<f32>, f32) {
    let normal = normalize(&inter.1.normal);
    let normal_l = dot(&normal, &ray.dir) * normal;
    let refraction = normalize(&-(n2 / n1 * (ray.dir - normal_l) + normal_l));
    (Ray::new(ray.origin + ray.dir * inter.1.toi, refraction), n2)
}
