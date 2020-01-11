use super::propagation::{Emission, Reception};
use byteorder::{LittleEndian, WriteBytesExt};
use specs::prelude::*;
use std::collections::HashMap;

pub struct TrackerSystem {
    files: HashMap<String, std::fs::File>,
}

impl TrackerSystem {
    pub fn new(names: Vec<String>) -> Self {
        let mut files = HashMap::new();
        std::fs::create_dir("output").ok();
        for name in names {
            files.insert(
                name.clone(),
                std::fs::File::create(format!("output/{}.bin", name)).expect("Invalid file"),
            );
        }
        Self { files }
    }
}

impl<'a> System<'a> for TrackerSystem {
    type SystemData = (ReadStorage<'a, Reception>, ReadStorage<'a, Emission>);

    fn run(&mut self, (recs, emits): Self::SystemData) {
        for rec in recs.join() {
            let mut file = self.files.get(&rec.label).expect("Name not found");
            file.write_f32::<LittleEndian>(rec.current)
                .expect("Failed to write receive value in Tracker");
        }

        for emit in emits.join() {
            let mut file = self.files.get(&emit.label).expect("Name not found");
            file.write_f32::<LittleEndian>(emit.current)
                .expect("Failed to write emit value in Tracker");
        }
    }
}
