use std::os::fd::{AsRawFd, RawFd};

use crate::motor_backend::{MotorBackend, MotorBackendIoPrep, MotorBackendParseError};
use std::time::{Duration, Instant};

pub struct MotorCtx<const R: usize, const W: usize> {
    pub config: MotorConfig,
    state: MotorState,
    pub(crate) backend: MotorBackend<R, W>,
}

use io_uring::squeue::Entry as SQEntry;
use io_uring::squeue::PushError as SQPushError;
use io_uring::types::Timespec;

use crate::controller::{Controller, ControllerState};

impl<const R: usize, const W: usize> AsRawFd for MotorCtx<R, W> {
    fn as_raw_fd(&self) -> RawFd {
        self.backend.as_raw_fd()
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub enum ControlState {
    Position {
        show_velocity: bool,
        show_torque: bool,
    },
    Velocity {
        show_position: bool,
        show_torque: bool,
    },
    Torque {
        show_position: bool,
        show_velocity: bool,
    },
}

pub struct MotorConfig {
    pub gear_reduction: f64,
    pub controller: Option<Controller>,
    pub state: ControlState,
}

#[derive(Clone, Copy, Default)]
pub struct CVP {
    pub current: f64,
    pub position: f64,
    pub velocity: f64,
}

enum MotorControl {
    Idle,
    Constant(CVP),
    Preset(Vec<(CVP, Duration)>)
}

struct MotorState {
    inputs: Vec<CVP>,
    outputs: Vec<CVP>,
    controller: Option<ControllerState>,
    last_update: Instant,
    input_state: MotorControl,
}

/*
impl MotorState {
    fn update<const R: usize, const W: usize>(
        &mut self,
        backend: &mut MotorBackend<R, W>,
        input: Option<CVP>,
        output: Option<CVP>,
        mode: &ControlState,
    ) -> MotorBackendIoPrep {
        // maybe good to have Optional input here?
        // then the backend can choose how to take empty input 
        //
        // or, have some way of sharing the existing motors between threads as to be able to
        // find which one to communicate with whenever a input is requested

        if let Some(mut input) = input {
            let mut now = std::time::Instant::now();
            core::mem::swap(&mut self.last_update, &mut now);
            let dt = self.last_update.duration_since(now);

            match mode {
                ControlState::Position { .. } => {
                    input.position =
                        self.update_controller(input.position, output.map(|cvp| cvp.position), dt)
                }
                ControlState::Velocity { .. } => {
                    input.velocity =
                        self.update_controller(input.velocity, output.map(|cvp| cvp.velocity), dt)
                }
                ControlState::Torque { .. } => {
                    input.current = self.update_controller(input.current, output.map(|cvp| cvp.current), dt)
                }
            };
            backend.prepare_input_msg(input, mode)
        } else {
            backend.prepare_idle_msg(mode)
        }
    }

    fn update_controller(&mut self, mut input: f64, output: Option<f64>, dt: Duration) -> f64 {
        match (&mut self.controller, output) {
            (Some(c), Some(o)) => c.update(input, o, dt),
            _ => input,
        }
    }
}
*/
