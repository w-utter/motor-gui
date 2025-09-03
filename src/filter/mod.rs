//! Filter module: common trait and type-erased wrapper.
//!
//! This module is self-contained and not wired into the existing UI/backend
//! to respect the constraint of not modifying existing files. External
//! consumers (tests or future integration) can use `FilterType` to construct
//! a `FilterChain` and process streaming samples with `push(x, dt)`.

mod ma;
mod ema;
mod butter;

pub use ema::ExponentialMA;
pub use ma::MovingAverage;
pub use butter::LowpassButter;

/// Common filter interface for streaming usage.
pub trait Filter {
    /// Push one sample with time delta in seconds; returns filtered output.
    fn push(&mut self, x: f64, dt: f32) -> f64;
    /// Reset internal state to initial conditions.
    fn reset(&mut self);
}

/// Supported filter types with parameters.
#[derive(Clone, Copy, Debug)]
pub enum FilterType {
    /// Moving average over a fixed window length (samples).
    MovingAverage { window: usize },
    /// Exponential moving average with constant `alpha` in (0, 1].
    ExponentialMA { alpha: f32 },
    /// Low-pass Butterworth by cutoff `fc_hz` and `order` in {1, 2}.
    LowpassButter { fc_hz: f32, order: u8 },
}

/// Type-erased holder for a single active filter.
///
/// Name uses "Chain" as an abstraction boundary — it can be extended to
/// sequence multiple filters if needed without changing the public API.
pub struct FilterChain {
    inner: FilterImpl,
}

enum FilterImpl {
    MA(MovingAverage),
    EMA(ExponentialMA),
    Butter(LowpassButter),
    Bypass,
}

impl FilterChain {
    /// Build a chain from a filter type. Invalid params fallback to Bypass.
    pub fn from_type(ft: FilterType) -> Self {
        use FilterImpl as FI;
        let inner = match ft {
            FilterType::MovingAverage { window } if window >= 1 => {
                FI::MA(MovingAverage::new(window))
            }
            FilterType::ExponentialMA { alpha } if alpha.is_finite() && alpha > 0.0 => {
                FI::EMA(ExponentialMA::new(alpha))
            }
            FilterType::LowpassButter { fc_hz, order } if fc_hz.is_finite() && fc_hz > 0.0 => {
                match order {
                    1 => FI::Butter(LowpassButter::one_pole(fc_hz)),
                    2 => FI::Butter(LowpassButter::butter2(fc_hz)),
                    _ => FI::Bypass,
                }
            }
            _ => FI::Bypass,
        };
        Self { inner }
    }

    /// Push one sample with `dt` (seconds) and return filtered value.
    pub fn push(&mut self, x: f64, dt: f32) -> f64 {
        match &mut self.inner {
            FilterImpl::MA(f) => f.push(x, dt),
            FilterImpl::EMA(f) => f.push(x, dt),
            FilterImpl::Butter(f) => f.push(x, dt),
            FilterImpl::Bypass => x,
        }
    }

    /// Reset state.
    pub fn reset(&mut self) {
        match &mut self.inner {
            FilterImpl::MA(f) => f.reset(),
            FilterImpl::EMA(f) => f.reset(),
            FilterImpl::Butter(f) => f.reset(),
            FilterImpl::Bypass => {}
        }
    }
}

