
pub trait EmitterController {
    fn emit(&mut self, buffer: &mut Vec<f32>);
    fn startup(&mut self);
}