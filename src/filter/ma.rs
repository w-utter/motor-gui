use super::Filter;

/// O(1) sliding window moving average with ring buffer and running sum.
pub struct MovingAverage {
    window: usize,
    buf: Vec<f64>,
    idx: usize,
    filled: usize,
    sum: f64,
}

impl MovingAverage {
    pub fn new(window: usize) -> Self {
        let w = window.max(1);
        Self {
            window: w,
            buf: vec![0.0; w],
            idx: 0,
            filled: 0,
            sum: 0.0,
        }
    }
}

impl Filter for MovingAverage {
    fn push(&mut self, x: f64, _dt: f32) -> f64 {
        if !x.is_finite() {
            // Drop invalid inputs; return current average
            let denom = self.filled.max(1) as f64;
            return self.sum / denom;
        }

        if self.filled < self.window {
            // Growing phase
            self.buf[self.idx] = x;
            self.sum += x;
            self.idx = (self.idx + 1) % self.window;
            self.filled += 1;
            return self.sum / (self.filled as f64);
        }

        // Steady-state phase
        let old = self.buf[self.idx];
        self.buf[self.idx] = x;
        self.idx = (self.idx + 1) % self.window;
        self.sum += x - old;
        self.sum / (self.window as f64)
    }

    fn reset(&mut self) {
        self.idx = 0;
        self.filled = 0;
        self.sum = 0.0;
        for v in &mut self.buf { *v = 0.0; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ma_step_response_reaches_one() {
        let mut ma = MovingAverage::new(4);
        let mut y = 0.0;
        for _ in 0..4 { y = ma.push(0.0, 0.001); }
        for _ in 0..4 { y = ma.push(1.0, 0.001); }
        assert!((y - 1.0).abs() < 1e-12);
    }
}

