pub mod refractive_indices {
    lazy_static! {
        pub static ref AIR: f32 = 1f32;
        pub static ref SNOW: f32 = 1.533f32.sqrt();
        pub static ref SOIL: f32 = 6f32.sqrt();
        pub static ref ROCK: f32 = 38f32.sqrt();
        pub static ref CONCRETE: f32 = 10f32.sqrt();
    }
}
