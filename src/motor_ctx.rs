use crate::controller::Controller;

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
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

#[derive(Debug)]
pub struct MotorConfig {
    pub gear_reduction: f64,
    pub controller: Option<Controller>,
    pub state: ControlState,
}

#[derive(Clone, Copy, Default, Debug)]
pub struct CVP {
    pub current: f64,
    pub position: f64,
    pub velocity: f64,
}
