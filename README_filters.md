# Signal Filters (V1)

This document describes a self-contained filter module for `motor-gui` that can be integrated later into the UI/backend. To honor the constraint of not modifying existing files, the module is added under `src/filter/` and exposed via `src/lib.rs`.

## Overview

- Module path: `motor_gui::filter`
- Core trait: `Filter` with `push(x, dt)` and `reset()`
- Types:
  - `FilterType`: `MovingAverage { window }`, `ExponentialMA { alpha }`, `LowpassButter { fc_hz, order }`
  - `FilterChain`: type-erased holder that runs one filter and can later be extended to chains
- Implementations:
  - Moving Average: ring buffer + running sum (O(1))
  - Exponential MA: first-order IIR with constant alpha
  - Lowpass Butterworth: order 1 (RC) and order 2 (biquad) via bilinear transform

## Usage

```rust
use motor_gui::filter::{FilterType, FilterChain, Filter};

let mut filt = FilterChain::from_type(FilterType::LowpassButter { fc_hz: 10.0, order: 2 });
let dt = 1.0 / 1000.0; // 1 kHz
let y = filt.push(0.5, dt as f32);
```

Notes:
- `dt` is in seconds; the biquad recomputes coefficients per call for robustness to jitter.
- Invalid parameters fall back to bypass (output = input).
- `reset()` reinitializes internal states; EMA and OnePole pass the very first sample after reset.

## Tests

Integration tests are added under `tests/filters.rs` and run against the library target exposed by `src/lib.rs`.

## Integration Plan (future)

1. Extend `motor_ctx` with `FilterConfig` and per-signal `FilterChain`s.
2. Compute `dt` from incoming samples in the backend event loop; call filter chain before pushing to plots.
3. Add a "Signal Filter" panel in the motor UI with enable toggles, type selection, and parameter inputs.
4. Plot overlays for raw/filtered and CSV export options.

## README Errata

The current README references `src/motor_backend/mod.rs`, but the codebase uses `src/motor_backend.rs`. Update the README accordingly when file edits are allowed.

