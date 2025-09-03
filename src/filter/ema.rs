use super::Filter;

/// Exponential moving average with constant alpha in (0,1].
pub struct ExponentialMA {
    alpha: f64,
    y: f64,
    initialized: bool,
}

impl ExponentialMA {
    pub fn new(alpha: f32) -> Self {
        let a = alpha as f64;
        let a = a.clamp(0.0, 1.0);
        Self { alpha: a, y: 0.0, initialized: false }
    }

    /// Update the smoothing factor; clamps to (0,1].
    pub fn set_alpha(&mut self, alpha: f32) {
        let a = (alpha as f64).clamp(0.0, 1.0);
        self.alpha = a;
    }
}

impl Filter for ExponentialMA {
    fn push(&mut self, x: f64, _dt: f32) -> f64 {
        if !x.is_finite() {
            return self.y;
        }
        if !self.initialized {
            self.y = x;
            self.initialized = true;
            return self.y;
        }
        let a = if self.alpha <= 0.0 { 1.0 } else { self.alpha };
        self.y = a * x + (1.0 - a) * self.y;
        self.y
    }

    fn reset(&mut self) {
        self.y = 0.0;
        self.initialized = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ema_is_monotonic_on_step() {
        let mut ema = ExponentialMA::new(0.2);
        let mut prev = ema.push(0.0, 0.001);
        for _ in 0..100 {
            let y = ema.push(1.0, 0.001);
            assert!(y >= prev - 1e-12);
            prev = y;
        }
        assert!((prev - 1.0).abs() < 1e-6);
    }
}

