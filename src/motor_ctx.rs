use crate::controller::{Controller};
// Filter configuration for each motor. Uses the library crate's filter types.
// This keeps UI/backend integration straightforward.
#[derive(Clone, Debug)]
pub struct FilterConfig {
    pub enabled: bool,
    pub velocity: bool,
    pub position: bool,
    pub current: bool,
    pub filter: motor_gui::filter::FilterType,
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            velocity: true,
            position: false,
            current: false,
            filter: motor_gui::filter::FilterType::ExponentialMA { alpha: 0.2 },
        }
    }
}

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
