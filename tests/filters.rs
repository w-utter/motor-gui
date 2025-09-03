use motor_gui::filter::{Filter, FilterChain, FilterType};

fn push_const(chain: &mut FilterChain, x: f64, dt: f32, n: usize) -> f64 {
    let mut y = 0.0;
    for _ in 0..n { y = chain.push(x, dt); }
    y
}

#[test]
fn ma_step_basic() {
    let mut fc = FilterChain::from_type(FilterType::MovingAverage { window: 8 });
    let _ = push_const(&mut fc, 0.0, 0.001, 8);
    let y = push_const(&mut fc, 1.0, 0.001, 8);
    assert!((y - 1.0).abs() < 1e-12);
}

#[test]
fn ema_monotone_step() {
    let mut fc = FilterChain::from_type(FilterType::ExponentialMA { alpha: 0.2 });
    let mut prev = fc.push(0.0, 0.001);
    for _ in 0..200 {
        let y = fc.push(1.0, 0.001);
        assert!(y >= prev - 1e-12);
        prev = y;
    }
    assert!((prev - 1.0).abs() < 1e-4);
}

#[test]
fn butter2_stable_and_convergent() {
    let mut fc = FilterChain::from_type(FilterType::LowpassButter { fc_hz: 10.0, order: 2 });
    for _ in 0..200 { let _ = fc.push(0.0, 0.001); }
    let mut last = 0.0;
    for _ in 0..2000 { last = fc.push(1.0, 0.001); assert!(last.is_finite()); }
    assert!((last - 1.0).abs() < 5e-3);
}

#[test]
fn reset_behavior_first_sample_passthrough() {
    let mut fc = FilterChain::from_type(FilterType::ExponentialMA { alpha: 0.5 });
    let _ = fc.push(0.0, 0.001);
    fc.reset();
    let y = fc.push(2.5, 0.001);
    assert!((y - 2.5).abs() < 1e-12);
}

#[test]
fn invalid_params_fallback_bypass() {
    let mut fc = FilterChain::from_type(FilterType::LowpassButter { fc_hz: -1.0, order: 7 });
    let y = fc.push(3.0, 0.01);
    assert_eq!(y, 3.0);
}

