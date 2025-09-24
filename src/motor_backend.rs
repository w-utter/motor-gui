pub(crate) mod ds402;
pub(crate) mod fourier;
pub(crate) mod protobuf;

use crate::motor_ctx::MotorConfig;
use std::time;

pub(crate) enum RequestedMotorInput {
    Cvp(crate::motor_ctx::CVP),
    Step(crate::StepInput),
    Impulse(crate::ImpulseInput),
    Custom(Vec<(crate::motor_ctx::CVP, time::Duration)>),
}

pub struct Backend<T> {
    motor_config: MotorConfig,
    input_cvp: Option<crate::motor_ctx::CVP>,
    request_input: Option<(std::time::Instant, RequestedMotorInput)>,
    backend_specific: T,
}

impl<T> Backend<T> {
    pub(crate) fn new(
        motor_config: MotorConfig,
        backend_specific: T,
        request_input: Option<RequestedMotorInput>,
    ) -> Self {
        Self {
            motor_config,
            input_cvp: None,
            backend_specific,
            request_input: request_input.map(|inp| (std::time::Instant::now(), inp)),
        }
    }
}
