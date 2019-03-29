pub mod RefractiveIndices {

    lazy_static! {
    pub static ref air: f32 = 1f32;
    pub static ref snow: f32 = 1.533f32.sqrt();
    pub static ref soil: f32 = 6f32.sqrt();
    pub static ref rock: f32 = 38f32.sqrt();
    pub static ref concrete: f32 = 10f32.sqrt();
}
}
