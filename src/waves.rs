use antennas::*;
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use ncollide3d::query::RayIntersectionCostFn;
use ncollide3d::shape::ShapeHandle;
use rayon::prelude;
use std;
use std::thread;

const NB_SAMPLE: u32 = 100;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;

pub type EnergyRay = (Ray<f32>, f32, f32);

pub fn tracing(world: &mut WorldDescriptor) {
    // TODO Use lifo
    let (s, r) = channel::unbounded();
    let (so, ro) = channel::unbounded();

    for (ide, x) in world.emitters.iter().enumerate() {
        let rays = emit(&x.position).map(|ray| (ray, x.max_power, 0.));
        for ray in rays {
            s.send((ide, ray));
        }
    }

    for _ in 0..4 {
        let r = r.clone();
        let s = s.clone();
        let so = so.clone();
        thread::spawn(move || {
            for (ide, ray) in r {
                let cs = find_receiver(&ray);
                for c in cs.into_iter().map(|(idr, pow, dist)| (ide, idr, pow, dist)) {
                    so.send(c);
                }
                for ray in next_rays(&ray) {
                    s.send((ide, ray));
                }
            }
        });
    }
    drop(s);

    for (ide, idr, pow, dist) in ro {
        world.emitters[ide].transfers[idr].push((pow, dist));
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

fn find_receiver(ray: &EnergyRay) -> Vec<(usize, f32, f32)> {
    unimplemented!()
}

fn next_rays(ray: &EnergyRay) -> Vec<EnergyRay> {
    unimplemented!()
}
