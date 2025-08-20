use std::time::Duration;
use crate::motor_ctx;

//TODO: add cache for controllers in the config
#[derive(Debug)]
pub enum Controller {}

impl Controller {
    pub fn update(&mut self, input: motor_ctx::CVP, output: motor_ctx::CVP, control_state: &motor_ctx::ControlState) -> motor_ctx::CVP {
        todo!()
    }
}

pub enum ControllerState {}

impl ControllerState {
    pub fn update(&mut self, input: motor_ctx::CVP, output: motor_ctx::CVP, control_state: &motor_ctx::ControlState) -> motor_ctx::CVP {
        todo!()
    }
}
