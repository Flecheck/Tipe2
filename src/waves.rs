use antennas::{WorldDescriptor, SignalEvent};
use crossbeam_channel as channel;
use nalgebra::{Point3, Vector3};
use ncollide3d::query::Ray;
use ncollide3d::query::RayIntersectionCostFn;
use ncollide3d::shape::ShapeHandle;
use rayon;
use rayon::prelude::{IndexedParallelIterator, IntoParallelIterator, ParallelIterator};
use std;

const NB_SAMPLE: u32 = 100;
const NB_SAMPLEF: f32 = NB_SAMPLE as f32;

const PI: f32 = std::f32::consts::PI;

/// (Ray,power,distance)
type EnergyRay = (Ray<f32>, f32, f32);

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

    rayon::spawn(move || {
        rays.into_iter().for_each(|x| process(x, &so));
    });

    for (ide, idr, time, dist) in ro {
        world.emitters[ide].transfers[idr].push(SignalEvent { time, gain: dist });
    }
}

fn process((ide, ray): (usize, (EnergyRay)), out: &channel::Sender<(usize, usize, u32, f32)>) {
    let cs = find_receiver(&ray);
    for c in cs
        .into_iter()
        .map(|(idr, time, dist)| (ide, idr, time, dist))
    {
        out.send(c);
    }

    next_rays(&ray)
        .into_par_iter()
        .map(|x| (ide, x))
        .for_each(|x| process(x, out));
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

fn find_receiver(ray: &EnergyRay) -> Vec<(usize, u32, f32)> {
    unimplemented!()
}

fn next_rays(ray: &EnergyRay) -> Vec<EnergyRay> {
    unimplemented!()
}
