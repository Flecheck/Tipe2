use specs::prelude::*;
use super::propagation::Reception;
use std::io;
use std::collections::HashMap;
use byteorder::{WriteBytesExt, LittleEndian};

pub struct TrackerSystem {
    files: HashMap<String, std::fs::File>,
}

impl TrackerSystem {
    pub fn new(names: Vec<String>) -> Self {
        let mut files = HashMap::new();
        std::fs::create_dir("output");
        for name in names {
            files.insert(name.clone(), std::fs::File::create(format!("output/{}.bin", name)).expect("File ded lol"));
        }
        Self {
            files,
        }
    }
}

impl<'a> System<'a> for TrackerSystem {
    type SystemData = ReadStorage<'a, Reception>;

    fn run(&mut self, recs: Self::SystemData) {
        //println!("tracker");
        for rec in recs.join() {
            let mut file = self.files.get(&rec.label).expect("Name not found");
            file.write_f32::<LittleEndian>(rec.current);
        }
        //println!("tracker done");
    }
}