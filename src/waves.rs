use antennas::*;
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use ncollide3d::shape::ShapeHandle;
use rayon::prelude;
use std;

const NB_SAMPLE: u32 = 100;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;

pub type EnergyRay = (Ray<f32>, f32, f32);

pub fn tracing(world: &mut WorldDescriptor) {
    let (sender, receiver) = channel::unbounded();
    for x in &world.emitters {
        let rays = emit(&x.position).map(|ray| (ray, x.max_power, 0.));
        for ray in rays {
            find_receiver(&ray);
            for ray in next_rays(&ray) {
                sender.send(ray);
            }
        }
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

fn find_receiver(ray: &EnergyRay) {}

fn next_rays(ray: &EnergyRay) -> Vec<Ray<f32>> {
    unimplemented!()
}
