// waves.rs
#![allow(dead_code)]
use crate::antennas::{SignalEvent, WorldDescriptor};
use crate::TIME_PER_BEAT;
use crate::WAVE_VELOCITY;
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use rayon;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use rayon::ThreadPoolBuilder;
use std;

use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::partitioning::BVT;

use crate::antennas::ClosestRayTOICostFn;
use crate::antennas::SceneObject;
use ncollide3d::query::RayIntersection;

use rand;

use crate::antennas::create_bvt_tuple_receiver;
use nalgebra::geometry::UnitQuaternion;
use nalgebra::{Isometry3, Translation3};

use ncollide3d::partitioning::BVH;
use ncollide3d::shape::Ball;

use std::mem;

use crate::constants::refractive_indices;

use std::collections::BTreeMap;

const NB_SAMPLE: u32 = 10_000;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;
const MIN_GAIN: f32 = 0.001;
const BOUNCE_MARGIN: f32 = 0.00001;
pub const ABSORBANCE_AIR: f32 = 0.0001;

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

/// Do the ray tracing and populate emitters with receivers
pub fn tracing(world: &mut WorldDescriptor) {
    let threadpool = ThreadPoolBuilder::new()
        .stack_size(64 * 1024usize.pow(2))
        .build()
        .unwrap();
    let ball = Ball::new(0.5f32);

    for (i, receiver) in world
        .receivers
        .iter()
        .enumerate()
        .filter_map(|(i, x)| x.as_ref().map(|x| (i, x)))
    {
        world.collisions.push(create_bvt_tuple_receiver(
            &ball,
            Isometry3::from_parts(
                Translation3::new(
                    receiver.position.x,
                    receiver.position.y,
                    receiver.position.z,
                ),
                UnitQuaternion::identity(),
            ),
            i,
        ))
    }

    let collisions = BVT::new_balanced(mem::replace(&mut world.collisions, vec![]));

    let (so, ro) = channel::bounded(10_000);

    threadpool.scope(|s| {
        let ref emitters = world.emitters;

        // Starting rays
        let rays = emitters
            .into_par_iter()
            .enumerate()
            .filter_map(|(i, x)| x.as_ref().map(|x| (i, x)))
            .flat_map(|(ide, x)| {
                emit((x.position).clone())
                    .map(move |ray| (ray.0, x.max_power * ray.1, 0.))
                    .map(move |ray| (ide, ray))
            });
        
        // Processing
        s.spawn(move |_s| {
            rays.map(|(ide, (ray, energy, distance))| {
                (
                    ide,
                    EnergyRay {
                        ray,
                        energy,
                        distance,
                        max_energy: energy,
                        n: *refractive_indices::AIR,
                    },
                )
            })
            .for_each(|x| {
                let ide = x.0;
                let mut ray = x.1;
                let mut iter = 0;
                while let Some((new_ray, new_iter)) = process((ide, ray), &so, &collisions, iter) {
                    ray = new_ray;
                    iter = new_iter;
                }
            });
        });

        // Collecting
        for out in ro {
            world.receivers[out.idr].as_mut().unwrap().transfers[out.ide].push(SignalEvent {
                time: out.time,
                gain: out.energy,
            });
        }
    });

    for receiver in world.receivers.iter_mut().filter_map(|x| x.as_mut()) {
        for transfers in &mut receiver.transfers {
            let res = transfers.iter().fold(BTreeMap::new(), |mut acc, x| {
                // Using BTreeMap to get vec sorted by time
                *acc.entry(x.time).or_default() += x.gain;
                acc
            });
            *transfers = res
                .iter()
                .map(|(&time, &gain)| SignalEvent { time, gain })
                .collect();
        }
    }
}

// TODO: Dephasage refraction
fn process(
    (ide, energyray): (usize, EnergyRay),
    out: &channel::Sender<Output>,
    bvs: &BVT<SceneObject, AABB<f32>>,
    rec: usize,
) -> Option<(EnergyRay, usize)> {
    if (energyray.energy / energyray.max_energy).abs() < MIN_GAIN {
        return None;
    }
    let mut visitor = ClosestRayTOICostFn::new(&energyray.ray);
    if let Some(inter) = bvs
        .best_first_search(&mut visitor)
        .map(|(id, r)| (bvs.content(id).1.expect("no data in node"), r))
    {
        let dist_plus = (energyray.ray.dir * inter.1.toi).norm() * energyray.n;
        let mut n2 = inter.0.n;
        let mut energy = energyray.energy;
        if n2 == energyray.n {
            n2 = *refractive_indices::AIR;
            energy = energy * (-inter.0.absorbance * dist_plus as f32).exp();
        } else {
            energy = energy * (-ABSORBANCE_AIR * dist_plus as f32).exp();
        }
        let n1 = energyray.n;

        let rand: f32 = rand::random();

        let normal = inter.1.normal.normalize();

        // If the ray crossed an antenna, record the encounter
        if let Some(idr) = inter.0.receiver {
            out.send(Output {
                ide,
                idr,
                time: ((energyray.distance + dist_plus) / (WAVE_VELOCITY * TIME_PER_BEAT)).floor()
                    as usize,
                energy: energy,
            });
        }

        let nextrays;

        if n2 / n1 > 1. {
            // Total reflection

            let reflection = next_rays_reflection(&energyray.ray, &inter);
            let normal_l = (normal.dot(&reflection.dir) * normal).normalize();

            nextrays = Some(EnergyRay {
                ray: reflection.translate_by(normal_l * BOUNCE_MARGIN),
                energy: -energy,
                distance: energyray.distance + dist_plus,
                max_energy: energyray.max_energy,
                n: n1,
            });
        } else {
            let cos1 = (normal.dot(&energyray.ray.dir) * normal - energyray.ray.dir).norm();
            let cos2 = (1. - (n1 / n2) * (n1 / n2) * (1. - cos1 * cos1).sqrt()).sqrt();

            let rtm = ((n1 * cos2 - n2 * cos1) / (n1 * cos2 + n2 * cos1)).abs();

            if rand < rtm {
                // Reflection
                {
                    let reflection = next_rays_reflection(&energyray.ray, &inter);
                    let normal_l = (normal.dot(&reflection.dir) * normal).normalize();

                    nextrays = Some(EnergyRay {
                        ray: reflection.translate_by(normal_l * BOUNCE_MARGIN),
                        energy: -energy,
                        distance: energyray.distance + dist_plus,
                        max_energy: energyray.max_energy,
                        n: n1,
                    });
                }
            } else {
                // Refraction
                let refraction = next_rays_refraction(&energyray.ray, &inter, energyray.n, n2);
                let normal_l = (normal.dot(&refraction.0.dir) * normal).normalize();

                nextrays = Some(EnergyRay {
                    ray: refraction.0.translate_by(normal_l * BOUNCE_MARGIN),
                    energy: energy,
                    distance: energyray.distance + dist_plus,
                    max_energy: energyray.max_energy,
                    n: n2,
                });
            }
        }

        let nextrays = nextrays.unwrap();
        /*if rec % 1000 == 0 && rec != 0 {
            println!("{}",dist_plus)
        }*/
        //if rec < 1000 {
        return Some((nextrays, rec + 1));
        //}
    }
    return None;
}

fn emit<'a>(pos: Point3<f32>) -> impl ParallelIterator<Item = (Ray<f32>, f32)> {
    (0..NB_SAMPLE).into_par_iter().flat_map(move |alpha| {
        (0..NB_SAMPLE).into_par_iter().map(move |beta| {
            let phi = alpha as f32 * 2f32 * PI / NB_SAMPLEF;
            let theta = beta as f32 * PI / NB_SAMPLEF;
            let x = theta.sin() * phi.cos();
            let y = theta.sin() * phi.sin();
            let z = theta.cos();
            (
                Ray::new(pos.clone(), Vector3::new(x, y, z).normalize()),
                2. * phi * (1. - (theta / 2.).cos()) / NB_SAMPLEF.powi(2),
            )
        })
    })
}

fn next_rays_reflection(ray: &Ray<f32>, inter: &(&SceneObject, RayIntersection<f32>)) -> Ray<f32> {
    let normal = inter.1.normal.normalize();

    let reflection = (2. * normal.dot(&ray.dir) * normal - ray.dir).normalize();

    Ray::new(ray.origin + ray.dir * inter.1.toi, reflection)
}

fn next_rays_refraction(
    ray: &Ray<f32>,
    inter: &(&SceneObject, RayIntersection<f32>),
    n1: f32,
    n2: f32,
) -> (Ray<f32>, f32) {
    let normal = inter.1.normal.normalize();
    let normal_l = normal.dot(&ray.dir) * normal;
    let refraction = (-(n2 / n1 * (ray.dir - normal_l)
        + normal * n2.powi(2) / n1.powi(2) * (1.0 - 1.0 / normal_l.norm().powi(2))))
    .normalize();
    (Ray::new(ray.origin + ray.dir * inter.1.toi, refraction), n2)
}
