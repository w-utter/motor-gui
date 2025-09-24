mod controller;
mod motor_backend;
mod motor_ctx;

fn main() -> eframe::Result {
    eframe::run_native(
        "",
        Default::default(),
        Box::new(|_cc| Ok(Box::new(AppState::new()))),
    )
}

pub use motor_backend::ds402::{Ds402Cmd, Ds402Response};
pub use motor_backend::fourier::{FourierCmd, FourierResponse};
pub use motor_backend::protobuf::{ProtobufCmd, ProtobufResponse};

struct AppState {
    motors: Vec<MotorUiConfig>,
    _fourier: std::thread::JoinHandle<std::io::Result<()>>,
    fourier_tx: std::sync::mpsc::Sender<(Ipv4Addr, FourierCmd)>,
    fourier_rx: std::sync::mpsc::Receiver<(Ipv4Addr, FourierResponse)>,

    network_ifs: sysinfo::Networks,
    ecat: Option<ChosenEcatNetwork>,

    protobuf_tx: std::sync::mpsc::Sender<(String, ProtobufCmd)>,
    protobuf_rx: std::sync::mpsc::Receiver<(String, ProtobufResponse)>,
    _protobuf: std::thread::JoinHandle<std::io::Result<()>>,
}

struct ChosenEcatNetwork {
    network_itf: String,
    driver: std::thread::JoinHandle<()>,
    ds402_tx: std::sync::mpsc::Sender<(usize, Ds402Cmd)>,
    ds402_rx: std::sync::mpsc::Receiver<(usize, Ds402Response)>,
}

impl AppState {
    fn new() -> Self {
        let (fourier_tx, thread_rx) = std::sync::mpsc::channel();
        let (thread_tx, fourier_rx) = std::sync::mpsc::channel();

        let fourier_handle =
            std::thread::spawn(|| motor_backend::fourier::event_loop(thread_rx, thread_tx));

        let (protobuf_tx, thread_rx) = std::sync::mpsc::channel();
        let (thread_tx, protobuf_rx) = std::sync::mpsc::channel();

        let protobuf_handle =
            std::thread::spawn(|| motor_backend::protobuf::event_loop(thread_rx, thread_tx));

        AppState {
            motors: vec![],
            _fourier: fourier_handle,
            fourier_tx,
            fourier_rx,
            network_ifs: sysinfo::Networks::new(),
            ecat: None,
            protobuf_tx,
            protobuf_rx,
            _protobuf: protobuf_handle,
        }
    }
}

use motor_ctx::ControlState;
use std::net::Ipv4Addr;

struct MotorUiConfig {
    backend: MotorUiBackendConfig,
    gear_reduction_storage: String,
    gear_reduction: f64,
    control_state: ControlState,
    control_state_cache: ControlStateCache,

    input: MotorInput,
    input_cache: MotorInputCache,
    output: Vec<(motor_ctx::CVP, std::time::Instant)>,
    ignore_motor_output: bool,
}

struct ControlStateCache(u8);

macro_rules! write_bits {
    ($this:expr, $off_1:expr, $off_2:expr, $bit_1:expr, $bit_2:expr) => {
        $this &= !((1 << $off_1) | (1 << $off_2));
        $this |= (u8::from($bit_1) << $off_1) | (u8::from($bit_2) << $off_2);
    };
}

macro_rules! read_bit {
    ($this:expr, $offset:expr) => {
        ($this & (1 << $offset)) > 0
    };
}

impl ControlStateCache {
    fn new() -> Self {
        Self(0)
    }

    fn pos_ctrl_vel(&self) -> bool {
        read_bit!(self.0, 0)
    }

    fn pos_ctrl_i(&self) -> bool {
        read_bit!(self.0, 1)
    }

    fn vel_ctrl_pos(&self) -> bool {
        read_bit!(self.0, 2)
    }

    fn vel_ctrl_i(&self) -> bool {
        read_bit!(self.0, 3)
    }

    fn i_ctrl_pos(&self) -> bool {
        read_bit!(self.0, 4)
    }

    fn i_ctrl_vel(&self) -> bool {
        read_bit!(self.0, 5)
    }

    fn set_pos_ctrl(&mut self, show_vel: bool, show_i: bool) {
        write_bits!(self.0, 0, 1, show_vel, show_i);
    }

    fn set_vel_ctrl(&mut self, show_pos: bool, show_i: bool) {
        write_bits!(self.0, 2, 3, show_pos, show_i);
    }

    fn set_i_ctrl(&mut self, show_pos: bool, show_vel: bool) {
        write_bits!(self.0, 4, 5, show_pos, show_vel);
    }
}

impl ControlState {
    fn display(&mut self, cache: &mut ControlStateCache, ui: &mut egui::Ui) -> bool {
        let prev = self.clone();

        ui.radio_value(
            self,
            ControlState::Position {
                show_velocity: cache.pos_ctrl_vel(),
                show_torque: cache.pos_ctrl_i(),
            },
            "position",
        );
        ui.radio_value(
            self,
            ControlState::Velocity {
                show_position: cache.vel_ctrl_pos(),
                show_torque: cache.vel_ctrl_i(),
            },
            "velocity",
        );
        ui.radio_value(
            self,
            ControlState::Torque {
                show_velocity: cache.i_ctrl_vel(),
                show_position: cache.i_ctrl_pos(),
            },
            "torque",
        );

        match self {
            Self::Position {
                show_velocity,
                show_torque,
            } => {
                ui.checkbox(show_velocity, "show velocity");
                ui.checkbox(show_torque, "show torque");
                cache.set_pos_ctrl(*show_velocity, *show_torque);
            }
            Self::Velocity {
                show_position,
                show_torque,
            } => {
                ui.checkbox(show_position, "show position");
                ui.checkbox(show_torque, "show torque");
                cache.set_vel_ctrl(*show_position, *show_torque);
            }
            Self::Torque {
                show_position,
                show_velocity,
            } => {
                ui.checkbox(show_position, "show position");
                ui.checkbox(show_velocity, "show velocity");
                cache.set_i_ctrl(*show_position, *show_velocity);
            }
        }
        core::mem::discriminant(&prev) != core::mem::discriminant(self)
    }
}

impl MotorUiConfig {
    fn new() -> Self {
        Self {
            backend: MotorUiBackendConfig::None,
            gear_reduction_storage: String::new(),
            gear_reduction: 1.,
            control_state: ControlState::Position {
                show_velocity: false,
                show_torque: false,
            },
            control_state_cache: ControlStateCache::new(),

            input: MotorInput::Idle,
            input_cache: MotorInputCache::default(),

            output: vec![],
            ignore_motor_output: false,
        }
    }

    fn display(
        &mut self,
        fourier_tx: &std::sync::mpsc::Sender<(Ipv4Addr, FourierCmd)>,
        ds402_tx: Option<&std::sync::mpsc::Sender<(usize, Ds402Cmd)>>,
        protobuf_tx: &std::sync::mpsc::Sender<(String, ProtobufCmd)>,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
    ) {
        let mut changed = false;
        changed |= self
            .control_state
            .display(&mut self.control_state_cache, ui);
        changed |= self
            .backend
            .display(fourier_tx, ds402_tx, protobuf_tx, &self.control_state, ui);
        ui.horizontal(|ui| {
            ui.label("gear reduction: ");
            if ui
                .text_edit_singleline(&mut self.gear_reduction_storage)
                .changed()
            {
                match self.gear_reduction_storage.parse::<f64>() {
                    Ok(new_gear_reduction) if new_gear_reduction != self.gear_reduction => {
                        self.gear_reduction = new_gear_reduction;
                        changed = true;
                    }
                    _ => (),
                }
            }
            ui.label(format!("current gear reduction: {}", self.gear_reduction));
        });

        changed |= self.input.display_options(&mut self.input_cache, ui, ctx);
        let _changed = changed;

        ui.vertical(|ui| match &self.backend {
            MotorUiBackendConfig::Fourier(config) => {
                if let Some(ip) = config.ip_addr() {
                    if ui.button("send to motor").clicked() {
                        self.output.clear();
                        self.ignore_motor_output = false;
                        let _ = fourier_tx.send((ip, FourierCmd::SetWaveForm(self.input.clone())));
                    }

                    if ui.button("stop").clicked() {
                        self.ignore_motor_output = true;
                        let _ = fourier_tx.send((ip, FourierCmd::StopWaveform));
                    }
                }
            }
            MotorUiBackendConfig::Ds402(config) => {
                if let (Some(tx), Some(idx)) = (ds402_tx, config.idx) {
                    if ui.button("send to motor").clicked() {
                        self.output.clear();
                        self.ignore_motor_output = false;
                        let _ = tx.send((idx, Ds402Cmd::SetWaveForm(self.input.clone())));
                    }

                    if ui.button("stop").clicked() {
                        self.ignore_motor_output = true;
                        let _ = tx.send((idx, Ds402Cmd::StopWaveform));
                    }
                }
            }
            MotorUiBackendConfig::Protobuf(config) => {
                if let Some(path) = config.path.as_ref() {
                    if ui.button("send to motor").clicked() {
                        self.output.clear();
                        self.ignore_motor_output = false;
                        let _ = protobuf_tx
                            .send((path.clone(), ProtobufCmd::SetWaveForm(self.input.clone())));
                    }
                }
            }
            _ => (),
        });

        self.display_output_graph(ui, ctx)
    }

    fn display_output_graph(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        let first_time = self
            .output
            .first()
            .map(|(_, time)| *time)
            .unwrap_or(std::time::Instant::now());

        let contents = vec![];
        let mut writer = csv::Writer::from_writer(contents);

        egui::Window::new("output window").show(ctx, |ui| {
            ui.vertical(|ui| {
                use egui_plot::{Line, PlotPoint, PlotPoints};

                let main_points = self
                    .output
                    .iter()
                    .map(|(cvp, time)| {
                        let time = time.duration_since(first_time).as_secs_f64();

                        PlotPoint::new(time, cvp.velocity)
                    })
                    .collect::<Vec<_>>();

                if ui.button("save as CSV").clicked() {
                    for PlotPoint { x, y } in &main_points {
                        writer
                            .write_record(&[format!("{x}"), format!("{y}")])
                            .unwrap();
                    }

                    let time = chrono::Utc::now().to_rfc3339();
                    if let Ok(mut file) = std::fs::File::create(format!(
                        "{}/velocity-graph-{time}.csv",
                        std::env::home_dir()
                            .unwrap_or(std::path::PathBuf::from("/"))
                            .display()
                    )) {
                        let mut buf = writer.into_inner().unwrap();
                        use std::io::Write;
                        file.write(&buf).unwrap();
                        buf.clear();
                    }
                }

                egui_plot::Plot::new("output").show(ui, |plot| {
                    plot.line(Line::new("vel", PlotPoints::Owned(main_points)));
                });
            });
        });

        let contents = vec![];
        let mut writer = csv::Writer::from_writer(contents);

        if matches!(
            self.control_state,
            ControlState::Velocity {
                show_position: true,
                ..
            }
        ) {
            egui::Window::new("output window pos").show(ctx, |ui| {
                ui.vertical(|ui| {
                    use egui_plot::{Line, PlotPoint, PlotPoints};

                    let sub_points_1 = self
                        .output
                        .iter()
                        .map(|(cvp, time)| {
                            let time = time.duration_since(first_time).as_secs_f64();

                            PlotPoint::new(time, cvp.position)
                        })
                        .collect::<Vec<_>>();

                    if ui.button("save as CSV").clicked() {
                        for PlotPoint { x, y } in &sub_points_1 {
                            (&mut writer)
                                .write_record(&[format!("{x}"), format!("{y}")])
                                .unwrap();
                        }

                        let time = chrono::Utc::now().to_rfc3339();
                        if let Ok(mut file) =
                            std::fs::File::create(format!("position-graph-{time}.csv"))
                        {
                            let mut buf = writer.into_inner().unwrap();
                            use std::io::Write;
                            file.write(&buf).unwrap();
                            buf.clear();
                        }
                    }

                    egui_plot::Plot::new("output").show(ui, |plot| {
                        plot.line(Line::new("pos", PlotPoints::Owned(sub_points_1)));
                    })
                })
            });
        }

        let contents = vec![];
        let mut writer = csv::Writer::from_writer(contents);

        if matches!(
            self.control_state,
            ControlState::Velocity {
                show_torque: true,
                ..
            }
        ) {
            egui::Window::new("output window current").show(ctx, |ui| {
                ui.vertical(|ui| {
                    use egui_plot::{Line, PlotPoint, PlotPoints};

                    let sub_points_2 = self
                        .output
                        .iter()
                        .map(|(cvp, time)| {
                            let time = time.duration_since(first_time).as_secs_f64();

                            PlotPoint::new(time, cvp.current)
                        })
                        .collect::<Vec<_>>();

                    if ui.button("save as CSV").clicked() {
                        for PlotPoint { x, y } in &sub_points_2 {
                            writer
                                .write_record(&[format!("{x}"), format!("{y}")])
                                .unwrap();
                        }

                        let time = chrono::Utc::now().to_rfc3339();
                        if let Ok(mut file) =
                            std::fs::File::create(format!("current-graph-{time}.csv"))
                        {
                            let mut buf = writer.into_inner().unwrap();
                            use std::io::Write;
                            file.write(&buf).unwrap();
                            buf.clear();
                        }
                    }

                    egui_plot::Plot::new("output").show(ui, |plot| {
                        plot.line(Line::new("current", PlotPoints::Owned(sub_points_2)));
                    })
                })
            });
        }

        if ui.button("reset start time").clicked() {
            //self.start_instant = std::time::Instant::now();
            self.output.clear();
        }
    }
}

//TODO: one thread per backend?

#[derive(Default)]
enum MotorUiBackendConfig {
    #[default]
    None,
    Fourier(motor_backend::fourier::MotorUiConfig),
    Ds402(motor_backend::ds402::MotorUiConfig),
    Protobuf(motor_backend::protobuf::MotorUiConfig),
}

impl MotorUiBackendConfig {
    fn display(
        &mut self,
        fourier_tx: &std::sync::mpsc::Sender<(Ipv4Addr, FourierCmd)>,
        ds402_tx: Option<&std::sync::mpsc::Sender<(usize, Ds402Cmd)>>,
        protobuf_tx: &std::sync::mpsc::Sender<(String, ProtobufCmd)>,
        control_state: &ControlState,
        ui: &mut egui::Ui,
    ) -> bool {
        ui.horizontal(|ui| {
            if !matches!(self, Self::None) && ui.add(egui::Button::new("clear config")).clicked() {
                *self = Self::None;
            }
            if ui
                .add(egui::RadioButton::new(
                    matches!(self, Self::Fourier(_)),
                    "Fourier",
                ))
                .clicked()
            {
                *self = Self::Fourier(Default::default());
            }
            if ui
                .add(egui::RadioButton::new(
                    matches!(self, Self::Ds402(_)),
                    "ds402 (ethercat)",
                ))
                .clicked()
            {
                *self = Self::Ds402(Default::default());
            }

            if ui
                .add(egui::RadioButton::new(
                    matches!(self, Self::Protobuf(_)),
                    "protobuf",
                ))
                .clicked()
            {
                *self = Self::Protobuf(Default::default());
            }
        });

        match self {
            Self::Fourier(config) => {
                let changed = config.display(ui);

                if let Some(addr) = &config.ip {
                    if config.added {
                        if ui.button("remove").clicked() {
                            let _ = fourier_tx.send((*addr, FourierCmd::Remove));
                            config.added = false;
                        }
                    } else {
                        if ui.button("add").clicked() {
                            let _ = fourier_tx.send((
                                *addr,
                                FourierCmd::Add(
                                    config.encoding,
                                    None,
                                    crate::motor_ctx::MotorConfig {
                                        gear_reduction: 1.,
                                        controller: None,
                                        state: control_state.clone(),
                                    },
                                ),
                            ));
                            config.added = true;
                        }
                    }
                }
                changed
            }
            Self::Ds402(config) => match ds402_tx {
                Some(tx) => {
                    let changed = config.display(ui);

                    if let Some(idx) = config.idx {
                        if config.added {
                            if ui.button("remove").clicked() {
                                let _ = tx.send((idx, Ds402Cmd::Remove));
                                config.added = false;
                            }
                        } else {
                            if ui.button("add").clicked() {
                                let _ = tx.send((
                                    idx,
                                    Ds402Cmd::Add(crate::motor_ctx::MotorConfig {
                                        gear_reduction: 1.,
                                        controller: None,
                                        state: control_state.clone(),
                                    }),
                                ));
                                config.added = true;
                            }
                        }
                    }
                    changed
                }
                None => {
                    ui.label("select a network interface for ethercat");
                    false
                }
            },
            Self::Protobuf(config) => {
                let changed = config.display(ui);

                if let Some(path) = &config.path {
                    if config.added {
                        if ui.button("remove").clicked() {
                            let _ = protobuf_tx.send((path.clone(), ProtobufCmd::Remove));
                            config.added = false;
                        }
                    } else {
                        if ui.button("add").clicked() {
                            match motor_backend::protobuf::RawDevice::from_path(
                                path,
                                config.baud_rate,
                            ) {
                                Ok(dev) => {
                                    let _ = protobuf_tx.send((
                                        path.clone(),
                                        ProtobufCmd::Add(
                                            crate::motor_ctx::MotorConfig {
                                                gear_reduction: 1.,
                                                controller: None,
                                                state: control_state.clone(),
                                            },
                                            dev,
                                        ),
                                    ));
                                    println!("good");
                                }
                                Err(e) => println!("err: {e:?}"),
                            }

                            config.added = true;
                        }
                    }
                }
                changed
            }
            _ => false,
        }
    }
}

use std::time::Duration;

#[derive(Clone, Debug)]
pub struct StepInput {
    delay: Duration,
    magnitude: f64,
    on_dur: Duration,
}

#[derive(Clone, Debug)]
pub struct ImpulseInput {
    magnitude: f64,
    delay: Duration,
}

impl Default for ImpulseInput {
    fn default() -> Self {
        Self {
            magnitude: 1.,
            delay: Duration::new(1, 0),
        }
    }
}

impl Default for StepInput {
    fn default() -> Self {
        Self {
            delay: Duration::new(1, 0),
            magnitude: 1.,
            on_dur: Duration::new(1, 0),
        }
    }
}

#[derive(Default)]
struct MotorInputCache {
    const_storage: String,
    constant: Option<f64>,

    step_delay_storage: String,
    step_mag_storage: String,
    step_dur_storage: String,
    step: Option<StepInput>,

    imp_delay_storage: String,
    imp_mag_storage: String,
    impulse: Option<ImpulseInput>,
    custom: Vec<(f64, Duration)>,
}

#[derive(Clone, Debug)]
pub enum MotorInput {
    Idle,
    Constant(f64),
    Step(StepInput),
    Impulse(ImpulseInput),
    Custom(Vec<(f64, Duration)>),
}

macro_rules! calc_y_bounds {
    ($y:expr) => {
        if $y == 0. {
            (-1., 1.)
        } else if $y < 0. {
            ($y, 0.)
        } else if $y > 0. {
            (0., $y)
        } else {
            unreachable!("invalid floating point repr");
        }
    };
}

impl MotorInput {
    fn display_options(
        &mut self,
        cache: &mut MotorInputCache,
        ui: &mut egui::Ui,
        ctx: &egui::Context,
    ) -> bool {
        let mut changed = false;
        if ui
            .add(egui::RadioButton::new(matches!(self, Self::Idle), "idle"))
            .clicked()
        {
            self.move_prev_input(Self::Idle, cache);
            changed = true;
        }

        if ui
            .add(egui::RadioButton::new(
                matches!(self, Self::Constant(_)),
                "constant",
            ))
            .clicked()
        {
            if !matches!(self, Self::Constant(_)) {
                if let Some(cached) = core::mem::take(&mut cache.constant) {
                    self.move_prev_input(Self::Constant(cached), cache);
                } else {
                    self.move_prev_input(Self::Constant(1.), cache);
                }
                changed = true;
            }
        }

        if ui
            .add(egui::RadioButton::new(
                matches!(self, Self::Step(_)),
                "step",
            ))
            .clicked()
        {
            if !matches!(self, Self::Step(_)) {
                if let Some(cached) = core::mem::take(&mut cache.step) {
                    self.move_prev_input(Self::Step(cached), cache);
                } else {
                    self.move_prev_input(Self::Step(Default::default()), cache);
                }
                changed = true;
            }
        }

        if ui
            .add(egui::RadioButton::new(
                matches!(self, Self::Impulse(_)),
                "impulse",
            ))
            .clicked()
        {
            if !matches!(self, Self::Impulse(_)) {
                if let Some(cached) = core::mem::take(&mut cache.impulse) {
                    self.move_prev_input(Self::Impulse(cached), cache);
                } else {
                    self.move_prev_input(Self::Impulse(Default::default()), cache);
                }
                changed = true;
            }
        }

        if ui
            .add(egui::RadioButton::new(
                matches!(self, Self::Custom(_)),
                "custom",
            ))
            .clicked()
        {
            if !matches!(self, Self::Custom(_)) {
                let cached = core::mem::take(&mut cache.custom);
                self.move_prev_input(Self::Custom(cached), cache);
                changed = true;
            }
        }

        match self {
            Self::Idle => (),
            Self::Constant(c) => {
                ui.label("magnitude:");
                if ui.text_edit_singleline(&mut cache.const_storage).changed() {
                    match cache.const_storage.parse::<f64>() {
                        Ok(new_const) if new_const != *c => {
                            *c = new_const;
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current magnitude: {}", c));
            }
            Self::Step(StepInput {
                delay,
                magnitude,
                on_dur,
            }) => {
                ui.label("delay:");
                if ui
                    .text_edit_singleline(&mut cache.step_delay_storage)
                    .changed()
                {
                    match cache
                        .step_delay_storage
                        .parse::<duration_string::DurationString>()
                    {
                        Ok(new_delay) if new_delay != *delay => {
                            *delay = new_delay.into();
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current delay: {:#?}", delay));
                ui.label("magnitude:");
                if ui
                    .text_edit_singleline(&mut cache.step_mag_storage)
                    .changed()
                {
                    match cache.step_mag_storage.parse::<f64>() {
                        Ok(new_mag) if new_mag != *magnitude => {
                            *magnitude = new_mag;
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current magnitude: {}", magnitude));
                ui.label("duration:");
                if ui
                    .text_edit_singleline(&mut cache.step_dur_storage)
                    .changed()
                {
                    match cache
                        .step_dur_storage
                        .parse::<duration_string::DurationString>()
                    {
                        Ok(new_dur) if new_dur != *on_dur => {
                            *on_dur = new_dur.into();
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current duration: {:#?}", on_dur));
            }
            Self::Impulse(ImpulseInput { magnitude, delay }) => {
                ui.label("delay:");
                if ui
                    .text_edit_singleline(&mut cache.imp_delay_storage)
                    .changed()
                {
                    match cache
                        .imp_delay_storage
                        .parse::<duration_string::DurationString>()
                    {
                        Ok(new_delay) if new_delay != *delay => {
                            *delay = new_delay.into();
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current delay: {:#?}", delay));
                ui.label("magnitude:");
                if ui
                    .text_edit_singleline(&mut cache.imp_mag_storage)
                    .changed()
                {
                    match cache.imp_mag_storage.parse::<f64>() {
                        Ok(new_mag) if new_mag != *magnitude => {
                            *magnitude = new_mag;
                            changed = true;
                            //TODO propagate changes to driver
                        }
                        _ => (),
                    }
                }
                ui.label(format!("current magnitude: {}", magnitude));
            }
            _ => (),
        }
        self.display_prelim_graph(ctx);

        changed
    }

    fn display_prelim_graph(&mut self, ctx: &egui::Context) {
        if matches!(self, Self::Idle) {
            return;
        }

        egui::Window::new("input window").show(ctx, |ui| {
            use egui_plot::{Line, PlotPoint, PlotPoints};
            match self {
                Self::Constant(c) => {
                    let pp1 = PlotPoint::new(0., *c);
                    let pp2 = PlotPoint::new(100., *c);
                    let points = [pp1, pp2];

                    let (miny, maxy) = calc_y_bounds!(*c * 2.);

                    egui_plot::Plot::new("prelim")
                        .default_x_bounds(0., 10.)
                        .default_y_bounds(miny, maxy)
                        .show(ui, |plot| {
                            plot.line(Line::new("const", PlotPoints::Borrowed(&points)));
                        });
                }
                Self::Step(StepInput {
                    delay,
                    magnitude,
                    on_dur,
                }) => {
                    let d1 = delay.as_secs_f64();
                    let d2 = on_dur.as_secs_f64();

                    let pp1 = PlotPoint::new(0., 0.);
                    let pp2 = PlotPoint::new(d1, 0.);
                    let pp3 = PlotPoint::new(d1, *magnitude);
                    let pp4 = PlotPoint::new(d1 + d2, *magnitude);
                    let points = [pp1, pp2, pp3, pp4];

                    let (miny, maxy) = calc_y_bounds!(*magnitude * 2.);

                    egui_plot::Plot::new("prelim")
                        .default_x_bounds(0., f64::max(d1 + d2, 1.))
                        .default_y_bounds(miny, maxy)
                        .show(ui, |plot| {
                            plot.line(Line::new("const", PlotPoints::Borrowed(&points)));
                        });
                }
                Self::Impulse(ImpulseInput { delay, magnitude }) => {
                    let d1 = delay.as_secs_f64();

                    let maxx = f64::max(d1 * 2., 1.);

                    let pp1 = PlotPoint::new(0., 0.);
                    let pp2 = PlotPoint::new(d1, 0.);
                    let pp3 = PlotPoint::new(d1, *magnitude);
                    let pp4 = PlotPoint::new(d1, 0.);
                    let pp5 = PlotPoint::new(maxx, 0.);
                    let points = [pp1, pp2, pp3, pp4, pp5];

                    let (miny, maxy) = calc_y_bounds!(*magnitude * 2.);

                    egui_plot::Plot::new("prelim")
                        .default_x_bounds(0., maxx)
                        .default_y_bounds(miny, maxy)
                        .show(ui, |plot| {
                            plot.line(Line::new("const", PlotPoints::Borrowed(&points)));
                        });
                }
                _ => (),
            }
        });
    }

    fn move_prev_input(&mut self, mut rhs: Self, cache: &mut MotorInputCache) {
        core::mem::swap(self, &mut rhs);
        match rhs {
            Self::Constant(c) => cache.constant = Some(c),
            Self::Step(s) => cache.step = Some(s),
            Self::Impulse(i) => cache.impulse = Some(i),
            Self::Custom(c) => cache.custom = c,
            _ => (),
        }
    }
}

impl eframe::App for AppState {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();
        egui::TopBottomPanel::top("top pannel").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.menu_button("ecat interface", |ui| {
                    self.network_ifs.refresh(true);

                    for (interface_name, network) in &self.network_ifs {
                        if ui.button(format!("[{interface_name}]")).clicked() {
                            match &self.ecat {
                                Some(ChosenEcatNetwork { network_itf, .. })
                                    if network_itf == interface_name =>
                                {
                                    ()
                                }
                                Some(ChosenEcatNetwork { ds402_tx, .. }) => {}
                                None => {
                                    let (main_tx, thread_rx) = std::sync::mpsc::channel();
                                    let (thread_tx, main_rx) = std::sync::mpsc::channel();

                                    let ifname = interface_name.to_owned();
                                    let handle = std::thread::spawn(|| {
                                        motor_backend::ds402::event_loop(
                                            thread_rx, thread_tx, ifname,
                                        );
                                    });

                                    self.ecat = Some(ChosenEcatNetwork {
                                        ds402_tx: main_tx,
                                        ds402_rx: main_rx,
                                        network_itf: interface_name.to_owned(),
                                        driver: handle,
                                    });
                                }
                            }
                        }
                    }
                });
            });

            if ui.button("add motor").clicked() {
                self.motors.push(MotorUiConfig::new())
            }

            while let Ok((rx_ip, msg)) = self.fourier_rx.try_recv() {
                if let Some(motor) = self.motors.iter_mut().find(|m| match m.backend {
                    MotorUiBackendConfig::Fourier(motor_backend::fourier::MotorUiConfig {
                        ip,
                        ..
                    }) => ip == Some(rx_ip),
                    _ => false,
                }) {
                    match msg {
                        FourierResponse::OutputCVP(cvp, time) => {
                            if !motor.ignore_motor_output {
                                motor.output.push((cvp, time));
                            }
                        }
                        FourierResponse::ControllerAdjustedCVP(_cvp, _time) => {}
                        FourierResponse::EndWaveform => {
                            motor.ignore_motor_output = true;
                        }
                        FourierResponse::Error(io) => {
                            println!("io error: {io}");
                        }
                        msg => println!("received from fourier: {msg:?}"),
                    }
                } else {
                    println!("received from fourier: {msg:?}");
                }
            }

            if let Some(ds402_rx) = self.ecat.as_ref().map(|ecat| &ecat.ds402_rx) {
                while let Ok((rx_idx, msg)) = ds402_rx.try_recv() {
                    if let Some(motor) = self.motors.iter_mut().find(|m| match m.backend {
                        MotorUiBackendConfig::Ds402(motor_backend::ds402::MotorUiConfig {
                            idx,
                            added,
                            ..
                        }) => added && idx == Some(rx_idx),
                        _ => false,
                    }) {
                        match msg {
                            Ds402Response::OutputCVP(cvp, time) => {
                                if !motor.ignore_motor_output {
                                    motor.output.push((cvp, time));
                                }
                            }
                            Ds402Response::ControllerAdjustedCVP(_cvp, _time) => {}
                            Ds402Response::EndWaveform => {
                                motor.ignore_motor_output = true;
                            }
                            msg => println!("received from fourier: {msg:?}"),
                        }
                    }
                }
            }

            while let Ok((rx_path, msg)) = self.protobuf_rx.try_recv() {
                if let Some(motor) = self.motors.iter_mut().find(|m| match &m.backend {
                    MotorUiBackendConfig::Protobuf(motor_backend::protobuf::MotorUiConfig {
                        path,
                        ..
                    }) => path.as_ref() == Some(&rx_path),
                    _ => false,
                }) {
                    match msg {
                        ProtobufResponse::OutputCVP(cvp, time) => {
                            if !motor.ignore_motor_output {
                                motor.output.push((cvp, time));
                            }
                        }
                        ProtobufResponse::ControllerAdjustedCVP(_cvp, _time) => {}
                        ProtobufResponse::EndWaveform => {
                            motor.ignore_motor_output = true;
                        }
                        ProtobufResponse::Error(io) => {
                            println!("io error: {io}");
                        }
                        msg => println!("received from protobuf: {msg:?}"),
                    }
                } else {
                    println!("received from protobuf: {msg:?}");
                }
            }

            for motor in &mut self.motors {
                motor.display(
                    &self.fourier_tx,
                    self.ecat.as_ref().map(|ecat| &ecat.ds402_tx),
                    &self.protobuf_tx,
                    ui,
                    ctx,
                )
            }
        });
    }
}
