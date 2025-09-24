include!(concat!(env!("OUT_DIR"), "/protos/mod.rs"));

use io_uring::squeue::Entry as SQEntry;
use io_uring::squeue::PushError as SQPushError;
use io_uring::types::Timespec;

use super::{Backend, RequestedMotorInput};

use std::os::fd::{AsRawFd, RawFd};

pub struct MotorUiConfig {
    pub(crate) path_buf_storage: String,
    pub(crate) path: Option<String>,
    pub(crate) baud_rate: Option<BaudRate>,
    pub(crate) added: bool,
}

use crate::motor_ctx::{ControlState, MotorConfig};
use std::sync::mpsc;
use std::time;

impl MotorUiConfig {
    pub(crate) fn display(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;

        let Self {
            path_buf_storage,
            path,
            added: _,
            baud_rate,
        } = self;

        ui.horizontal(|ui| {
            ui.label("motor dev path");
            if ui.text_edit_singleline(path_buf_storage).changed() {
                match (&path, std::fs::exists(&path_buf_storage)) {
                    (Some(old), Ok(true)) if old != path_buf_storage => {
                        *path = Some(path_buf_storage.clone())
                    }
                    (None, Ok(true)) => *path = Some(path_buf_storage.clone()),
                    _ => (),
                }

                if path_buf_storage.is_empty() {
                    *path = None;
                }
            }

            if let Some(path) = &path {
                ui.label(format!("current path: {path:?}"));
            }
        });

        ui.menu_button("baud rate", |ui| {
            for rate in BaudRate::enumerate() {
                if ui.button(format!("{:?}", rate)).clicked() {
                    *baud_rate = Some(*rate)
                }
            }
        });

        if let Some(rate) = &self.baud_rate {
            ui.label(format!("set baud rate: {:?}", rate));
        }

        changed
    }

    pub fn path(&self) -> Option<&str> {
        self.path.as_ref().map(|s| s.as_ref())
    }
}

impl Default for MotorUiConfig {
    fn default() -> Self {
        Self {
            path_buf_storage: String::new(),
            path: None,
            added: false,
            baud_rate: Default::default(),
        }
    }
}

#[derive(Debug)]
pub enum ProtobufCmd {
    Add(MotorConfig, RawDevice),
    Remove,
    SetWaveForm(crate::MotorInput),
    StopWaveform,
    Shutdown,
}

#[derive(Debug)]
pub enum ProtobufResponse {
    OutputCVP(crate::motor_ctx::CVP, time::Instant),
    ControllerAdjustedCVP(crate::motor_ctx::CVP, time::Instant),
    Error(std::io::Error),
    Timeout,
    DuplicateConnections,
    EndWaveform,
}

pub fn event_loop(
    cmd_rx: mpsc::Receiver<(String, ProtobufCmd)>,
    err_tx: mpsc::Sender<(String, ProtobufResponse)>,
) -> std::io::Result<()> {
    let mut ring = io_uring::IoUring::new(16)?;
    use std::collections::HashMap;
    let mut connections = HashMap::new();
    let mut path_to_connection = std::collections::BTreeMap::new();
    let mut connection_to_path = HashMap::new();

    let mut probe = io_uring::register::Probe::new();
    ring.submitter().register_probe(&mut probe)?;

    use io_uring::opcode::{ReadMulti, Write};
    use io_uring::types;

    if !probe.is_supported(ReadMulti::CODE) || !probe.is_supported(Write::CODE) {
        panic!("opcodes not supported");
    }

    const ENTRIES: u32 = 32;
    const SIZE: u32 = 64;

    let mut idx = 10;

    loop {
        while let Some((path, cmd)) = match cmd_rx.try_recv() {
            Ok(cmd) => Some(cmd),
            Err(mpsc::TryRecvError::Disconnected) => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    "disconnected",
                ));
            }
            _ => None,
        } {
            match cmd {
                ProtobufCmd::Add(config, dev) => {
                    if connections.contains_key(&dev.as_raw_fd()) {
                        println!("duplicate");
                        continue;
                    }

                    let mut rx_bufs =
                        io_uring_buf_ring::BufRing::new(ENTRIES as _, SIZE as _, idx)?
                            .register(&ring.submitter());

                    while let Err((e, mut buf)) = rx_bufs {
                        if e.kind() != std::io::ErrorKind::AlreadyExists {
                            return Err(e);
                        }
                        idx += 1;
                        buf.set_bgid(idx);
                        rx_bufs = buf.register(&ring.submitter());
                    }

                    let rx_bufs = unsafe { rx_bufs.unwrap_unchecked() }.init();

                    let dev_id = dev.as_raw_fd();

                    use std::os::fd::AsRawFd;
                    let rx_multi_entry =
                        ReadMulti::new(types::Fd(dev.as_raw_fd()), 0, rx_bufs.bgid())
                            .build()
                            .user_data(dev_id as _);

                    use motor::{MotorDriver, motor_driver::Motor_cmd};
                    use protobuf::Message;

                    let mut cmd = MotorDriver::new();
                    cmd.motor_cmd = Some(Motor_cmd::Velocity(0));

                    let buf = cmd.write_to_bytes().unwrap();

                    let write_entry =
                        Write::new(types::Fd(dev.as_raw_fd()), buf.as_ptr(), buf.len() as _)
                            .build()
                            .user_data(dev_id as _)
                            .flags(io_uring::squeue::Flags::SKIP_SUCCESS);

                    while unsafe { ring.submission().push(&rx_multi_entry).is_err() } {
                        ring.submit().expect("could not submit ops");
                    }

                    while unsafe { ring.submission().push(&write_entry).is_err() } {
                        ring.submit().expect("could not submit ops");
                    }

                    ring.submit().expect("could not submit ops");

                    connections.insert(
                        dev_id,
                        Backend::new(config, ProtobufBackend { dev, buf: rx_bufs }, None),
                    );
                    path_to_connection.insert(path.clone(), dev_id);
                    connection_to_path.insert(dev_id, path.clone());
                }
                ProtobufCmd::Remove => {
                    if let Some(id) = path_to_connection.remove(&path) {
                        connections.remove(&id);
                        connection_to_path.remove(&id);
                    }
                }
                ProtobufCmd::SetWaveForm(waveform) => {
                    let Some(id) = path_to_connection.get(&path) else {
                        continue;
                    };
                    if let Some(motor) = connections.get_mut(id) {
                        use crate::MotorInput;
                        match waveform {
                            MotorInput::Constant(c) => {
                                let cvp = crate::motor_ctx::CVP {
                                    velocity: c,
                                    position: 0.,
                                    current: 0.,
                                };

                                motor.input_cvp = Some(cvp);
                                motor.request_input =
                                    Some((time::Instant::now(), RequestedMotorInput::Cvp(cvp)))
                            }
                            MotorInput::Idle => {
                                let cvp = crate::motor_ctx::CVP::default();
                                motor.input_cvp = Some(cvp);
                                motor.request_input =
                                    Some((time::Instant::now(), RequestedMotorInput::Cvp(cvp)));
                            }
                            MotorInput::Step(s) => {
                                let cvp = if s.delay <= time::Duration::ZERO {
                                    crate::motor_ctx::CVP {
                                        velocity: s.magnitude,
                                        position: 0.,
                                        current: 0.,
                                    }
                                } else {
                                    crate::motor_ctx::CVP::default()
                                };

                                motor.input_cvp = Some(cvp);
                                motor.request_input =
                                    Some((time::Instant::now(), RequestedMotorInput::Step(s)))
                            }
                            MotorInput::Impulse(i) => {
                                let cvp = if i.delay < time::Duration::ZERO {
                                    crate::motor_ctx::CVP {
                                        velocity: i.magnitude,
                                        position: 0.,
                                        current: 0.,
                                    }
                                } else {
                                    crate::motor_ctx::CVP::default()
                                };

                                motor.input_cvp = Some(cvp);
                                motor.request_input =
                                    Some((time::Instant::now(), RequestedMotorInput::Impulse(i)))
                            }
                            MotorInput::Custom(c) => {
                                let cvps = c
                                    .into_iter()
                                    .map(|(mag, time)| {
                                        let cvp = crate::motor_ctx::CVP {
                                            velocity: mag,
                                            position: 0.,
                                            current: 0.,
                                        };
                                        (cvp, time)
                                    })
                                    .collect::<Vec<_>>();

                                let cvp =
                                    cvps.iter().next().map(|(cvp, _)| *cvp).unwrap_or_default();

                                motor.input_cvp = Some(cvp);
                                motor.request_input =
                                    Some((time::Instant::now(), RequestedMotorInput::Custom(cvps)))
                            }
                        }
                    }
                }
                ProtobufCmd::StopWaveform => {
                    let Some(id) = path_to_connection.get(&path) else {
                        continue;
                    };
                    if let Some(motor) = connections.get_mut(id) {
                        motor.input_cvp = Some(crate::motor_ctx::CVP {
                            velocity: 0.,
                            position: 0.,
                            current: 0.,
                        });
                        let _ = err_tx.send((path, ProtobufResponse::EndWaveform));
                    }
                }

                _ => (),
            }
            /*
            match cmd {
                FourierCmd::Shutdown => {
                    request_shutdown = true;
                }
                _ => (),
            }
            */
        }

        let mut completed = ring.completion().next();

        while let Some(entry) = completed {
            completed = ring.completion().next();

            let id = entry.user_data() as i32;

            let Some(motor) = connections.get_mut(&id) else {
                continue;
            };

            let Some(name) = connection_to_path.get(&id) else {
                continue;
            };

            let result = entry.result();

            if result >= 0 {
                let Ok(Some(id)) = motor.backend_specific.buf.buffer_id_from_cqe(&entry) else {
                    continue;
                };

                let buf = id.buffer();

                let Ok(response) = motor::MotorDriverResponse::parse_from_bytes(buf) else {
                    continue;
                };
                drop(id);

                let cvp = crate::motor_ctx::CVP {
                    position: response.encoder_position.unwrap_or_default() as _,
                    velocity: response.encoder_velocity.unwrap_or_default() as _,
                    current: response.current.unwrap_or_default() as _,
                };

                motor.update_input(&err_tx, name);

                if let Some(controller) = &mut motor.motor_config.controller {
                    let input = motor.input_cvp.unwrap_or_default();

                    let new_input = controller.update(input, cvp, &motor.motor_config.state);
                    let _ = err_tx.send((
                        name.clone(),
                        ProtobufResponse::ControllerAdjustedCVP(new_input, time::Instant::now()),
                    ));
                    motor.input_cvp = Some(new_input);
                } else {
                    let _ = err_tx.send((
                        name.clone(),
                        ProtobufResponse::OutputCVP(cvp, time::Instant::now()),
                    ));
                }

                use motor::{MotorDriver, motor_driver::Motor_cmd};
                use protobuf::Message;

                let mut cmd = MotorDriver::new();
                cmd.motor_cmd = Some(Motor_cmd::Velocity(
                    motor.input_cvp.unwrap_or_default().velocity.round() as _,
                ));

                let buf = cmd.write_to_bytes().unwrap();

                let write_entry = Write::new(
                    types::Fd(motor.backend_specific.dev.as_raw_fd()),
                    buf.as_ptr(),
                    buf.len() as _,
                )
                .build()
                .user_data(motor.backend_specific.dev.as_raw_fd() as _)
                .flags(io_uring::squeue::Flags::SKIP_SUCCESS);

                while unsafe { ring.submission().push(&write_entry).is_err() } {
                    ring.submit().expect("could not submit ops");
                }

                ring.submit().expect("could not submit ops");
            } else {
                println!("err: {result}");
            }
        }
    }
}

impl Backend<ProtobufBackend> {
    fn update_input(&mut self, end_tx: &mpsc::Sender<(String, ProtobufResponse)>, path: &str) {
        let next_input = match &self.request_input {
            None => None,
            Some((_, RequestedMotorInput::Cvp(c))) => Some(*c),
            Some((then, RequestedMotorInput::Step(s))) => {
                let now = time::Instant::now();
                let elapsed = now.duration_since(*then);

                if elapsed > s.delay + s.on_dur {
                    let _ = end_tx.send((path.to_string(), ProtobufResponse::EndWaveform));
                    None
                } else if elapsed > s.delay {
                    Some(crate::motor_ctx::CVP {
                        velocity: s.magnitude,
                        ..Default::default()
                    })
                } else {
                    Some(Default::default())
                }
            }
            Some((then, RequestedMotorInput::Impulse(i))) => {
                let now = time::Instant::now();
                let elapsed = now.duration_since(*then);

                const EPSILON: time::Duration = time::Duration::from_millis(10);

                if elapsed > i.delay + i.delay {
                    let _ = end_tx.send((path.to_string(), ProtobufResponse::EndWaveform));
                    None
                } else if elapsed < i.delay + EPSILON && elapsed > i.delay - EPSILON {
                    Some(crate::motor_ctx::CVP {
                        velocity: i.magnitude,
                        ..Default::default()
                    })
                } else {
                    Some(Default::default())
                }
            }
            Some((then, RequestedMotorInput::Custom(c))) => {
                let now = time::Instant::now();
                let elapsed = now.duration_since(*then);

                let mut total_dur = time::Duration::ZERO;

                let cvp = c.iter().find_map(|(input_cvp, dur)| {
                    total_dur += *dur;
                    if elapsed < total_dur {
                        Some(*input_cvp)
                    } else {
                        None
                    }
                });

                if cvp.is_none() {
                    let _ = end_tx.send((path.to_string(), ProtobufResponse::EndWaveform));
                }
                cvp
            }
        };

        self.input_cvp = next_input;
    }
}

#[derive(Debug)]
pub struct RawDevice {
    fd: std::os::fd::OwnedFd,
}

impl AsRawFd for RawDevice {
    fn as_raw_fd(&self) -> i32 {
        self.fd.as_raw_fd()
    }
}

impl RawDevice {
    pub fn from_path(path: &str, baud_rate: Option<BaudRate>) -> std::io::Result<Self> {
        let fd: std::os::fd::OwnedFd = std::fs::File::options()
            .read(true)
            .write(true)
            .open(path)
            .map(Into::into)?;

        if unsafe { libc::fcntl(fd.as_raw_fd(), libc::F_SETFL, libc::O_NDELAY) } < 0 {
            return Err(std::io::Error::last_os_error());
        }

        init_tty(fd.as_raw_fd(), baud_rate)?;

        Ok(Self { fd })
    }
}

fn init_tty(fd: i32, baud_rate: Option<BaudRate>) -> std::io::Result<libc::termios> {
    let mut term = unsafe { core::mem::MaybeUninit::uninit() };

    if unsafe { libc::tcgetattr(fd, term.as_mut_ptr()) } < 0 {
        return Err(std::io::Error::last_os_error());
    }

    let mut term = unsafe { term.assume_init() };

    term.c_cflag |= libc::CREAD | libc::CLOCAL;
    term.c_cflag &= !libc::CSTOPB;

    unsafe {
        libc::cfmakeraw(&mut term);
    }

    term.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);

    if let Some(rate) = baud_rate {
        let rate = rate.as_const();

        if unsafe { libc::cfsetispeed(&mut term, rate) } < 0 {
            return Err(std::io::Error::last_os_error());
        }

        if unsafe { libc::cfsetospeed(&mut term, rate) } < 0 {
            return Err(std::io::Error::last_os_error());
        }
    }

    if unsafe { libc::tcsetattr(fd, libc::TCIFLUSH, &term) } < 0 {
        return Err(std::io::Error::last_os_error());
    }

    Ok(term)
}

macro_rules! baud_rate {
    ($($rate:ident),*) => {
        #[derive(PartialEq, Eq, Clone, Copy)]
        pub enum BaudRate {
            $(
                $rate,
            )*
        }

        impl BaudRate {
            fn as_const(&self) -> u32 {
                match self {
                    $(
                        Self::$rate => libc::$rate,
                    )*
                }
            }

            fn enumerate() -> &'static [Self] {
                &[$(Self::$rate,)*]
            }
        }

        impl core::fmt::Debug for BaudRate {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    $(
                        Self::$rate => {
                            let rate = stringify!($rate);
                            write!(f, "{}", &rate[1..])?;
                        }
                    )*
                }
                Ok(())
            }
        }
    }
}

baud_rate! {
    B0,
    B50,
    B75,
    B110,
    B134,
    B150,
    B200,
    B300,
    B600,
    B1200,
    B1800,
    B2400,
    B4800,
    B9600,
    B19200,
    B38400,
    B57600,
    B921600,
    B1000000,
    B1152000,
    B1500000,
    B2000000,
    B2500000,
    B3000000,
    B3500000,
    B4000000
}

struct ProtobufBackend {
    dev: RawDevice,
    buf: io_uring_buf_ring::BufRing<io_uring_buf_ring::buf_ring_state::Init>,
}
