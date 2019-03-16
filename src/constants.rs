mod RefractiveIndices {

    lazy_static! {
    static ref air: f32 = 1f32;
    static ref snow: f32 = 1.533f32.sqrt();
    static ref soil: f32 = 6f32.sqrt();
    static ref rock: f32 = 38f32.sqrt();
    static ref concrete: f32 = 10f32.sqrt();
}
}
