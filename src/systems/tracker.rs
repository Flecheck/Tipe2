use super::propagation::{Emission, Reception};
use byteorder::{LittleEndian, WriteBytesExt};
use specs::prelude::*;
use std::collections::HashMap;
use std::io;

pub struct TrackerSystem {
    files: HashMap<String, std::fs::File>,
}

impl TrackerSystem {
    pub fn new(names: Vec<String>) -> Self {
        let mut files = HashMap::new();
        std::fs::create_dir("output");
        for name in names {
            files.insert(
                name.clone(),
                std::fs::File::create(format!("output/{}.bin", name)).expect("File ded lol"),
            );
        }
        Self { files }
    }
}

impl<'a> System<'a> for TrackerSystem {
    type SystemData = (ReadStorage<'a, Reception>, ReadStorage<'a, Emission>);

    fn run(&mut self, (recs, emits): Self::SystemData) {
        //println!("tracker");
        for rec in recs.join() {
            let mut file = self.files.get(&rec.label).expect("Name not found");
            file.write_f32::<LittleEndian>(rec.current);
        }

        for emit in emits.join() {
            let mut file = self.files.get(&emit.label).expect("Name not found");
            file.write_f32::<LittleEndian>(emit.current);
        }
        //println!("tracker done");
    }
}
