use super::propagation::{Emission, Reception};
use antennas::SignalEvent;
use bit_vec::BitVec;
use rayon::iter::IntoParallelIterator;
use rustfft::{num_complex::Complex, num_traits::Zero, FFT, FFTplanner};
use specs::{Component, Entity, ReadStorage, System, Join, VecStorage, WriteStorage};
use std::collections::VecDeque;
use std::sync::Arc;
use std::f32::consts::PI;

const CARRIER_GROUP_SIZE: usize = 8;
const SYMBOL_DURATION: usize = 2048;

fn generate_wavetables() -> [[f32; SYMBOL_DURATION]; CARRIER_GROUP_SIZE] {
    let mut res = [[0.0; SYMBOL_DURATION]; CARRIER_GROUP_SIZE];
    let df = 1.0 / SYMBOL_DURATION as f32;
    for k in 0..CARRIER_GROUP_SIZE {
        for i in 0..SYMBOL_DURATION {
            res[k][i] = (2.0 * PI * df * ((k + 1) as f32) * (i as f32)).sin();
        }
    }
    res
}

pub struct OFDMEmitter {
    data_buffer: [BitVec; CARRIER_GROUP_SIZE], // Splitted data to transmit
    buffer_pos: usize,
    clock: u32,
    wavetables: [[f32; SYMBOL_DURATION]; CARRIER_GROUP_SIZE],
    is_current_phased: [bool; CARRIER_GROUP_SIZE],
}

impl OFDMEmitter {
    pub fn new(signal: &[u8]) -> OFDMEmitter {
        let mut global_vec = BitVec::from_bytes(signal);

        while global_vec.len() % CARRIER_GROUP_SIZE != 0 {
            global_vec.push(false);
        }

        println!("{:?}", global_vec);

        let mut data_buffer: [BitVec; CARRIER_GROUP_SIZE] = Default::default();

        for k in 0..CARRIER_GROUP_SIZE {
            data_buffer[k] = BitVec::with_capacity(global_vec.len() / CARRIER_GROUP_SIZE);
        }

        let mut pos = 0;
        while pos < global_vec.len() {
            for k in 0..CARRIER_GROUP_SIZE {
                data_buffer[k].push(global_vec[pos]);
                pos += 1;
            }
        }

        OFDMEmitter {
            data_buffer,
            buffer_pos: 0,
            wavetables: generate_wavetables(),
            clock: 0,
            is_current_phased: [false; CARRIER_GROUP_SIZE],
        }
    }
}

impl Component for OFDMEmitter {
    type Storage = VecStorage<OFDMEmitter>;
}

pub struct OFDMReceiver {
    pub data_buffer: BitVec,

    clock: u32,
    began: bool,
    before_start: u32,

    symbol_buffer: [Complex<f32>; SYMBOL_DURATION],
    symbol_result: [Complex<f32>; SYMBOL_DURATION],
    fft: Arc<dyn FFT<f32>>,
}

impl OFDMReceiver {
    pub fn new() -> OFDMReceiver {
        OFDMReceiver {
            data_buffer: BitVec::new(),
            clock: 0,
            before_start: 0,
            began: false,
            symbol_buffer: [Complex::zero(); SYMBOL_DURATION],
            symbol_result: [Complex::zero(); SYMBOL_DURATION],
            fft: FFTplanner::new(false).plan_fft(SYMBOL_DURATION),
        }
    }
}

impl Component for OFDMReceiver {
    type Storage = VecStorage<OFDMReceiver>;
}

pub struct OFDMEmit;

impl<'a> System<'a> for OFDMEmit {
    type SystemData = (WriteStorage<'a, Emission>, WriteStorage<'a, OFDMEmitter>);

    fn run(&mut self, (mut emission, mut ofdme): Self::SystemData) {
        (&mut emission, &mut ofdme).join().for_each(|(emit, oe)| {
            if oe.clock == 0 {
                if oe.buffer_pos == oe.data_buffer[0].len() {
                    emit.current = 0.0;
                    return;
                }

                // Prepare the next symbol
                for i in 0..CARRIER_GROUP_SIZE {
                    oe.is_current_phased[i] = oe.data_buffer[i][oe.buffer_pos];
                }

                oe.buffer_pos += 1;
            }

            emit.current = 0.0;
            for i in 0..CARRIER_GROUP_SIZE {
                emit.current += if oe.is_current_phased[i] {
                    oe.wavetables[i][oe.clock as usize]
                } else {
                    -oe.wavetables[i][oe.clock as usize]
                };
            }

            oe.clock += 1;
            if oe.clock == SYMBOL_DURATION as u32 {
                oe.clock = 0;
            }
        });
    }
}

pub struct OFDMReceive;

impl<'a> System<'a> for OFDMReceive {
    type SystemData = (WriteStorage<'a, Reception>, WriteStorage<'a, OFDMReceiver>);

    fn run(&mut self, (mut reception, mut ofdmr): Self::SystemData) {
        (&mut reception, &mut ofdmr).join().for_each(|(rec, or)| {
            if !or.began {
                if rec.current != 0.0 {
                    println!("Starts at {}", or.before_start);
                    or.began = true;
                } else {
                    or.before_start += 1;
                    return;
                }
            }

            or.symbol_buffer[or.clock as usize] = Complex::new(rec.current, 0.0);

            or.clock += 1;

            if or.clock == SYMBOL_DURATION as u32 {
                // We received a complete symbol, we need to process it

                or.fft.process(&mut or.symbol_buffer, &mut or.symbol_result);

                // Quick recap on the output of the FFT
                // output[k] is Ae^(iϕ) for f = k/SYMBOL_DURATION
                // which is pretty handy as we are working with OFDM
                // so subcarriers are orthogonal: all subcarriers
                // frequencies are of the pattern k/SYMBOL_DURATION
                // We can have up to SYMBOL_DURATION subcarriers here

                for k in 0..CARRIER_GROUP_SIZE {
                    // If the real part of output[k] is positive
                    // then phi ~= 0, else phi ~= -pi
                    if or.symbol_result[k+1].re > 0.0 {
                        or.data_buffer.push(false);
                    } else {
                        or.data_buffer.push(true);
                    }
                }

                or.clock = 0;
            }
        });
    }
}