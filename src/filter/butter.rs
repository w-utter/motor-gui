use super::Filter;

const PI: f64 = std::f64::consts::PI;

/// Low-pass Butterworth filters (order 1 or 2) with runtime dt.
///
/// - Order 1: classic RC low-pass with tau = 1/(2π fc)
/// - Order 2: biquad via RBJ cookbook (Butterworth, Q = 1/√2) using bilinear transform
pub enum LowpassButter {
    OnePole { fc_hz: f64, y: f64, init: bool },
    Biquad2 { fc_hz: f64, // parameters
              // state
              x1: f64, x2: f64, y1: f64, y2: f64 },
    Disabled,
}

impl LowpassButter {
    pub fn one_pole(fc_hz: f32) -> Self { Self::OnePole { fc_hz: fc_hz as f64, y: 0.0, init: false } }
    pub fn butter2(fc_hz: f32) -> Self { Self::Biquad2 { fc_hz: fc_hz as f64, x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0 } }
}

impl Filter for LowpassButter {
    fn push(&mut self, x: f64, dt: f32) -> f64 {
        if !x.is_finite() { return self.last(); }
        let dt = dt as f64;
        let dt = dt.clamp(1e-6, 1.0); // Clamp dt for stability

        match self {
            LowpassButter::OnePole { fc_hz, y, init } => {
                let fc = (*fc_hz).max(1e-6);
                let tau = 1.0 / (2.0 * PI * fc);
                let alpha = dt / (tau + dt);
                if !*init { *y = x; *init = true; }
                else { *y += alpha * (x - *y); }
                *y
            }
            LowpassButter::Biquad2 { fc_hz, x1, x2, y1, y2 } => {
                let fc = (*fc_hz).max(1e-6);
                let fs = 1.0 / dt;
                let k = (PI * fc / fs).tan();
                // Butterworth Q
                let q = 1.0_f64 / 2.0_f64.sqrt();
                let norm = 1.0 / (1.0 + k / q + k * k);
                let b0 = k * k * norm;
                let b1 = 2.0 * b0;
                let b2 = b0;
                let a1 = 2.0 * (k * k - 1.0) * norm;
                let a2 = (1.0 - k / q + k * k) * norm;

                // Direct Form I difference equation
                let y0 = b0 * x + b1 * *x1 + b2 * *x2 - a1 * *y1 - a2 * *y2;

                *x2 = *x1; *x1 = x;
                *y2 = *y1; *y1 = y0;
                if y0.is_finite() { y0 } else { 0.0 }
            }
            LowpassButter::Disabled => x,
        }
    }

    fn reset(&mut self) {
        match self {
            LowpassButter::OnePole { y, init, .. } => { *y = 0.0; *init = false; }
            LowpassButter::Biquad2 { x1, x2, y1, y2, .. } => { *x1 = 0.0; *x2 = 0.0; *y1 = 0.0; *y2 = 0.0; }
            LowpassButter::Disabled => {}
        }
    }
}

impl LowpassButter {
    fn last(&self) -> f64 {
        match self {
            LowpassButter::OnePole { y, .. } => *y,
            LowpassButter::Biquad2 { y1, .. } => *y1,
            LowpassButter::Disabled => 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_pole_tracks_step_without_instability() {
        let mut lp = LowpassButter::one_pole(5.0);
        let mut y = 0.0;
        for _ in 0..200 { y = lp.push(0.0, 0.001); }
        for _ in 0..2000 { y = lp.push(1.0, 0.001); }
        assert!((y - 1.0).abs() < 1e-3);
    }

    #[test]
    fn butter2_is_bounded_with_dt_jitter() {
        let mut lp = LowpassButter::butter2(10.0);
        let mut y = 0.0;
        let mut dt = 0.001_f32;
        for i in 0..5000 {
            // jitter dt +-10%
            let phase = (i % 20) as f32;
            dt = 0.001 * (1.0 + 0.1 * ((phase - 10.0) / 10.0));
            y = lp.push(if i < 1000 { 0.0 } else { 1.0 }, dt);
            assert!(y.is_finite());
            assert!(y > -10.0 && y < 10.0);
        }
    }
}

