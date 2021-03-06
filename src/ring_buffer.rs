// ring_buffer.rs

/// Ringbuffer-like queue datastructure with O(1) pop and arbitrary get.
pub struct RingBuffer<T> {
    buffer: Vec<T>,
    reader: usize, // Modular offset in the buffer
}

impl<T> RingBuffer<T>
where
    T: Default + Clone,
{
    pub fn with_capacity(capacity: usize) -> Self {
        if capacity == 0 {
            panic!("Capacity should be strictly positive");
        }

        let mut buffer = Vec::with_capacity(capacity);
        buffer.resize(capacity, T::default());
        Self { buffer, reader: 0 }
    }

    pub fn pop(&mut self) -> T {
        let elem = {
            let in_buf = unsafe { self.buffer.get_unchecked_mut(self.reader) };
            std::mem::replace(in_buf, T::default())
        };

        // Optimized modular increment
        // <=> self.reader = (self.reader + 1) % self.buffer.len();
        self.reader += 1;
        if self.reader == self.buffer.len() {
            self.reader = 0;
        }

        elem
    }

    pub fn get(&self, index: usize) -> Option<&T> {
        if index < self.buffer.len() {
            let index = self.reader + index;
            if index >= self.buffer.len() {
                // If the target is too far up, loop back.
                let index = index - self.buffer.len();
                self.buffer.get(index)
            } else {
                self.buffer.get(index)
            }
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if index < self.buffer.len() {
            let index = self.reader + index;
            if index >= self.buffer.len() {
                // If the target is too far up, loop back.
                let index = index - self.buffer.len();
                self.buffer.get_mut(index)
            } else {
                self.buffer.get_mut(index)
            }
        } else {
            None
        }
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }
}
