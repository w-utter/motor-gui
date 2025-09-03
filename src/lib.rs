// Library entry to expose newly added modules without modifying existing code paths.
// This lets integration tests import `motor_gui::filter` while keeping the
// current binary `src/main.rs` unchanged.

pub mod filter;

