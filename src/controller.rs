use crate::motor_ctx;

//TODO: add cache for controllers in the config
#[derive(Debug)]
pub enum Controller {}

impl Controller {
    pub fn update(&mut self, _input: motor_ctx::CVP, _output: motor_ctx::CVP, _control_state: &motor_ctx::ControlState) -> motor_ctx::CVP {
        todo!()
    }
}
