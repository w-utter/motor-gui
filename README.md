# motor-gui

Desktop GUI for communicating with a motor controller over the Amber AIOS protocol. Configure input waveforms (constant/step/impulse/custom), visualize real‑time feedback (velocity/position/current), and export CSV data.

## Project Overview

- Name: motor-gui (Rust + egui desktop app)
- Purpose: interact with a motor controller using Amber AIOS, send input waveforms, and plot returned CVP (current/velocity/position).
- Architecture: GUI with `eframe/egui`; device I/O runs in a dedicated thread powered by `io_uring`; messaging between GUI and backend via `mpsc`; device protocol by `amber-aios` (JSON or binary encoding).

## Features

- Multi-motor: add multiple motor panels via the top bar “add motor”.
- Backend selection: current backend is `Fourier`, with IPv4 and encoding (JSON/Binary) settings.
- Control modes: Position / Velocity / Torque (with optional display of related quantities).
- Input waveforms: Idle, Constant, Step, Impulse, Custom (sequence of magnitude + duration). Switching types preserves previous parameters via cache.
- Real-time plots: velocity as the main plot; optional position/current sub-plots depending on the chosen mode. “reset start time” clears the current data window.
- CSV export: each plot window has “save as CSV” to export current series.
- Device control: “send to motor” applies current input; “stop” halts waveform. Backend thread reports timeout and error events.

## Directory Layout

- `src/main.rs`: app entry, UI state, plotting, waveform editing, and message exchange with backend.
- `src/controller.rs`: controller interface placeholder (TODO).
- `src/motor_ctx.rs`: domain models (control mode, CVP, config).
- `src/motor_backend.rs`: backend module entry.
- `src/motor_backend/fourier.rs`: Fourier backend (amber-aios + io_uring event loop).

## Tech Stack

- GUI: `eframe = 0.31`, `egui = 0.31`, `egui_plot = 0.32`.
- I/O: `io-uring` (Linux only).
- Protocol: `amber-aios` (Git dependency) with JSON/binary encoding.
- Helpers: `duration-string` for human-readable durations, `libc` for error codes.

## Environment

- OS: Linux with `io_uring` support (kernel 5.10+ recommended).
- Rust: 2024 edition (latest stable toolchain recommended).
- Network: controller reachable on the same network; required UDP ports exposed as defined by `amber-aios`.

## Build & Run

1) Ensure Rust is installed (e.g., via `rustup`).
2) Fetch dependencies (requires network for Git deps like `amber-aios` and `io-uring`).
3) Run:

```bash
cargo run            # debug
cargo run --release  # release
```

On first launch, click the top “add motor” button to start configuring a device.

## Usage

1) Add motor: click “add motor” and choose backend `Fourier`.
2) Configure device: enter IPv4 address; pick encoding (JSON/Binary).
3) Create connection: click “add” after address is set; once added, “remove” disconnects.
4) Select control mode: Position / Velocity / Torque, with optional display toggles.
5) Edit waveform:
   - Constant: set magnitude.
   - Step: set `delay`, `magnitude`, `duration` (accepts `1s`, `500ms`, etc.).
   - Impulse: set `delay` and `magnitude`.
   - Custom: provide multiple `(magnitude, duration)` segments.
6) Send/stop: “send to motor” applies the current input; “stop” halts the waveform.
7) View/export: open velocity/position/current windows to inspect plots; click “save as CSV” to export.

## Backend Event Loop

- Channels: GUI sends `(Ipv4Addr, FourierCmd)`; backend responds with `(Ipv4Addr, FourierResponse)`.
- io_uring: links `Send/Recv/LinkTimeout` entries; builds requests per encoding; parses responses into `CVP`.
- Time-based waveforms: advances Step/Impulse/Custom by timestamps; emits `EndWaveform` on completion.
- Error/timeout: on timeout/error, re-queues I/O and reports via responses.

## Known Issues & Notes

- Missing dependencies: code uses `csv` and `chrono` but `Cargo.toml` does not list them. Add under `[dependencies]`:

  ```toml
  csv = "1"
  chrono = "0.4"
  ```

- Paths & permissions: velocity CSV uses `std::env::home_dir()` (deprecated); other exports write to the current directory. Ensure write permissions.
- Controller logic: `controller.rs` -> `Controller::update` is `todo!()`; currently no closed-loop adjustments, raw device output is shown.
- Platform: backend requires Linux (`io_uring`); other platforms will not run the backend.
- Error display: detailed errors are printed to console; not surfaced in the UI yet.

## Development Tips

- Plotting: `egui_plot` with `PlotPoints` for multiple lines; separate floating windows for each plot.
- New backends: add a module under `motor_backend`, mirror `fourier.rs`, and implement the `Backend<T>` hooks and UI bindings.
- Performance: I/O queue depth is 16; serialization is handled by `amber-aios`. Adjust `FourierBackend::<R, W>` buffer sizes if needed.
