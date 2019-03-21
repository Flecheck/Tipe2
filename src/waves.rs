#![allow(dead_code)]
use antennas::{SignalEvent, WorldDescriptor};
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use rayon;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use std;
use WAVE_VELOCITY;
use TIME_PER_BEAT;

use ncollide3d::bounding_volume::aabb::AABB;
use ncollide3d::partitioning::BVT;

use antennas::ClosestRayTOICostFn;
use antennas::SceneObject;
use ncollide3d::query::RayIntersection;

use rand;

use nalgebra::{dot, norm, normalize};

use antennas::create_bvt_tuple_receiver;
use nalgebra::{Isometry3,Translation3};
use nalgebra::geometry::UnitQuaternion;

use ncollide3d::shape::Ball;

use std::mem;

const NB_SAMPLE: u32 = 10;
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
    let ball = Ball::new(0.05f32);
    
    for (i,receiver) in world.receivers.iter().enumerate() {
        world.collisions.push(create_bvt_tuple_receiver(&ball,Isometry3::from_parts(Translation3::new(receiver.position.x,receiver.position.y,receiver.position.z) ,UnitQuaternion::identity()), i))
    }

    let collisions = BVT::new_balanced(mem::replace(&mut world.collisions, vec![]));

    let (so, ro) = channel::bounded(10_000);

    rayon::scope(|s| {
        let ref emitters = world.emitters;

        // Starting rays
        let rays = emitters.into_par_iter().enumerate().flat_map(|(ide, x)| {
            emit((x.position).clone())
                .map(move |ray| (ray.0, x.max_power * ray.1, 0.))
                .map(move |ray| (ide, ray))
        });
        // Prosessing
        s.spawn(move |_s| {
            rays.map(|(ide, (ray, energy, distance))| {
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
            .for_each(|x| process(x, &so, &collisions));
        });

        // Collecting
        for out in ro {
            world.receivers[out.idr].transfers[out.ide].push(SignalEvent {
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

        let mut nextrays = None;

        let rand: f32 = rand::random();

        if let Some(idr) = inter.0.receiver {
            println!("dist_plus: {}", dist_plus);
            out.send(Output {
                ide,
                idr,
                time: (energyray.distance + dist_plus / (WAVE_VELOCITY * TIME_PER_BEAT)).floor() as usize,
                energy: energyray.energy,
            })
        } else {
            if n2 / n1 > 1. {
                // Reflection
                {
                    let reflection = next_rays_reflection(&energyray.ray, &inter);

                    nextrays = Some(EnergyRay {
                        ray: reflection.translate_by(reflection.dir * 0.001),
                        energy: energyray.energy,
                        distance: energyray.distance + dist_plus,
                        max_energy: energyray.max_energy,
                        n: n1,
                    });
                }
            }

            // Refraction
            if n2 / n1 <= 1. {
                let normal = normalize(&inter.1.normal);

                let cos1 = norm(&(dot(&normal, &energyray.ray.dir) * normal - energyray.ray.dir));
                let cos2 = (1. - (n1 / n2) * (n1 / n2) * (1. - cos1 * cos1).sqrt()).sqrt();

                let rtm = ((n1 * cos2 - n2 * cos1) / (n1 * cos2 + n2 * cos1)).abs();

                if rand < rtm {
                    // Reflection
                    {
                        let reflection = next_rays_reflection(&energyray.ray, &inter);

                        nextrays = Some(EnergyRay {
                            ray: reflection.translate_by(reflection.dir * 0.001),
                            energy: energyray.energy,
                            distance: energyray.distance + dist_plus,
                            max_energy: energyray.max_energy,
                            n: n1,
                        });
                    }
                } else {
                    let refraction = next_rays_refraction(&energyray.ray, &inter, energyray.n, n2);
                    nextrays = Some(EnergyRay {
                        ray: refraction.0.translate_by(refraction.0.dir * 0.001),
                        energy: energyray.energy,
                        distance: energyray.distance + dist_plus,
                        max_energy: energyray.max_energy,
                        n: n2,
                    });
                }
            }

            let nextrays =
                nextrays.expect("WTFFFFFFFFFFFFFFFFFFF pas de reflection ou de refraction");
            process((ide, nextrays), out, bvs);
        }
    }
}

fn emit<'a>(pos: Point3<f32>) -> impl ParallelIterator<Item = (Ray<f32>, f32)> {
    (0..NB_SAMPLE).into_par_iter().flat_map(move |alpha| {
        (0..NB_SAMPLE).into_par_iter().map(move |beta| {
            let phi = alpha as f32 * 2f32 * PI / NB_SAMPLEF;
            let theta = beta as f32 * 2f32 * PI / NB_SAMPLEF;
            let x = theta.sin() * phi.cos();
            let y = theta.sin() * phi.sin();
            let z = theta.sin();
            (
                Ray::new(pos.clone(), normalize(&Vector3::new(x, y, z))),
                2. * phi * (1. - (theta / 2.).cos()),
            )
        })
    })
}

fn next_rays_reflection(ray: &Ray<f32>, inter: &(&SceneObject, RayIntersection<f32>)) -> Ray<f32> {
    let normal = normalize(&inter.1.normal);

    let reflection = normalize(&(2. * dot(&normal, &ray.dir) * normal - ray.dir));

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
