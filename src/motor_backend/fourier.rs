use io_uring::squeue::Entry as SQEntry;
use io_uring::squeue::PushError as SQPushError;
use io_uring::types::Timespec;

use super::{Backend, RequestedMotorInput};

use std::os::fd::{AsRawFd, RawFd};

pub(crate) struct FourierConfig {
    pub encode: FourierEncodeKind,
    pub read_timeout: Option<Timespec>,
}

use std::net::Ipv4Addr;

pub struct MotorUiConfig {
    pub(crate) ip_buf_storage: String,
    pub(crate) ip: Option<Ipv4Addr>,
    pub(crate) encoding: FourierEncodeKind,
    pub(crate) added: bool,
}

impl MotorUiConfig {
    pub(crate) fn display(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;

        let Self {
            ip_buf_storage,
            ip,
            encoding,
            added: _,
        } = self;

        ui.horizontal(|ui| {
            ui.label("motor ip addr");
            if ui.text_edit_singleline(ip_buf_storage).changed() {
                match (&ip, ip_buf_storage.parse::<IpAddr>()) {
                    (_, Ok(IpAddr::V6(_))) => todo!(),
                    (Some(old_ip), Ok(IpAddr::V4(new_ip))) if new_ip != *old_ip => {
                        // signal to the driver to remove the old ip and create the new ip.
                        *ip = Some(new_ip);
                        changed = true;
                    }
                    (None, Ok(IpAddr::V4(new_ip))) => {
                        // create a new ip
                        *ip = Some(new_ip);
                        changed = true;
                    }
                    _ => (),
                }

                if ip_buf_storage.is_empty() {
                    *ip = None;
                }
            }

            if let Some(ip) = &ip {
                ui.label(format!("current ip: {ip}"));
            }
        });

        ui.horizontal(|ui| {
            ui.label(format!("communications encoding"));
            use crate::motor_backend::fourier::FourierEncodeKind;
            ui.radio_value(encoding, FourierEncodeKind::Json, "JSON");
            ui.radio_value(encoding, FourierEncodeKind::Binary, "binary");
        });

        changed
    }

    pub fn ip_addr(&self) -> Option<Ipv4Addr> {
        self.ip
    }
}

impl Default for MotorUiConfig {
    fn default() -> Self {
        Self {
            ip_buf_storage: String::new(),
            ip: None,
            encoding: FourierEncodeKind::Binary,
            added: false,
        }
    }
}

const DEFAULT_FOURIER_READ_TIMEOUT: Timespec = Timespec::new().nsec(1000000000);

pub(crate) struct FourierSendRecv {
    pub send: SQEntry,
    pub recv: SQEntry,
    pub recv_timeout: SQEntry,
}

impl FourierSendRecv {
    pub fn queue(&self, q: &mut io_uring::SubmissionQueue<'_>) -> Result<(), SQPushError> {
        let Self {
            send,
            recv,
            recv_timeout,
        } = self;

        unsafe {
            q.push(&send)?;
            q.push(&recv)?;
            q.push(&recv_timeout)?;
        }
        Ok(())
    }
}

pub(crate) struct FourierBackend<const R: usize, const W: usize> {
    motor: amber_aios::AiosMotor<R, W>,
    config: FourierConfig,
    enabled: bool,
    remove_next_recv: bool,
}

use crate::motor_ctx::{ControlState, MotorConfig};

impl<const R: usize, const W: usize> FourierBackend<R, W> {
    pub fn new(
        ip_addr: Ipv4Addr,
        encode: FourierEncodeKind,
        read_timeout: Option<Timespec>,
    ) -> Result<Self, amber_aios::Err> {
        let motor = amber_aios::AiosMotor::from_addr(ip_addr)?;

        let config = FourierConfig {
            encode: encode,
            read_timeout,
        };

        Ok(Self {
            motor,
            config,
            enabled: false,
            remove_next_recv: false,
        })
    }

    pub fn prepare_input_msg(
        &mut self,
        cvp: crate::motor_ctx::CVP,
        mode: &ControlState,
        f: impl FnOnce(FourierSendRecv) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        let set_pos = amber_aios::cmds::binary::set_input_position(
            cvp.position as _,
            cvp.velocity as _,
            cvp.current as _,
        );
        let set_pos_json =
            amber_aios::cmds::set_position::<1>(cvp.position, cvp.velocity as _, cvp.current as _);

        let set_vel =
            amber_aios::cmds::binary::set_input_velocity(cvp.velocity as _, cvp.current as _);
        let set_vel_json = amber_aios::cmds::set_velocity::<1>(cvp.velocity, cvp.current);

        let set_i = amber_aios::cmds::binary::set_input_torque(cvp.current as _);
        let set_i_json = amber_aios::cmds::set_current::<1>(cvp.current);

        let enable_cmd = amber_aios::cmds::set_requested_state::<1>(amber_aios::AxisState::Enable);

        if !self.enabled {
            self.prepare_msg_from_cmd(&enable_cmd, f)
        } else {
            match (mode, &self.config.encode) {
                (ControlState::Position { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_pos_json, f)
                }
                (ControlState::Position { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_pos, f)
                }
                (ControlState::Velocity { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_vel_json, f)
                }
                (ControlState::Velocity { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_vel, f)
                }
                (ControlState::Torque { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_i_json, f)
                }
                (ControlState::Torque { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_i, f)
                }
            }
        }
    }

    pub fn prepare_msg_from_cmd<'a, 'b, C: amber_aios::cmds::SerializableCommand<'a>>(
        &'b mut self,
        cmd: &'b C,
        f: impl FnOnce(FourierSendRecv) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        let motor = &mut self.motor;
        let fd = motor.as_raw_fd();
        let addr = motor.sock_addr(C::PORT);

        use io_uring::opcode::{LinkTimeout, Recv, Send};
        use io_uring::squeue::Flags;
        use io_uring::types::Fd;
        let send = {
            let send_buf = motor.serialize_cmd(cmd).expect("serialization error");

            Send::new(Fd(fd), send_buf.as_ptr(), send_buf.len() as _)
                .dest_addr(addr.as_ptr())
                .dest_addr_len(addr.len())
                .build()
                .user_data((motor.addr().to_bits() as u64) | 1 << 32)
                .flags(io_uring::squeue::Flags::SKIP_SUCCESS)
        };

        let recv = {
            let recv_buf = motor.read_buf_mut();

            Recv::new(Fd(fd), recv_buf.as_mut_ptr(), recv_buf.len() as _)
                .build()
                .flags(Flags::IO_LINK)
                .user_data(motor.addr().to_bits() as u64)
        };

        let read_timeout = self
            .config
            .read_timeout
            .unwrap_or(DEFAULT_FOURIER_READ_TIMEOUT);

        let recv_timeout = LinkTimeout::new(&read_timeout)
            .build()
            .user_data(motor.addr().to_bits() as u64);

        f(FourierSendRecv {
            send,
            recv,
            recv_timeout,
        })
    }

    pub fn parse_cvp(
        &mut self,
        len: usize,
        _config: &MotorConfig,
    ) -> Result<Option<crate::motor_ctx::CVP>, amber_aios::Err> {
        use amber_aios::cmds::Command;
        use amber_aios::cmds::binary::BinaryCommand;
        let bytes = &self.motor.read_buf()[..len];

        if !self.enabled {
            if let Ok(amber_aios::Request {
                data: amber_aios::RequestedState { current_state: 8 },
                ..
            }) = amber_aios::cmds::GetRequestedState::parse_return(bytes)
            {
                self.enabled = true;
                return Ok(None);
            } else {
                return Err(amber_aios::Err::UnexpectedReturn);
            }
        }

        let amber_aios::CVP {
            position,
            velocity,
            current,
        } = match &self.config.encode {
            FourierEncodeKind::Json => amber_aios::cmds::GetCVP::parse_return(bytes)?.data,
            FourierEncodeKind::Binary => {
                amber_aios::cmds::binary::BinGetCVP::parse_return(bytes)?.into()
            }
        };

        Ok(Some(crate::motor_ctx::CVP {
            position: position,
            velocity: velocity,
            current,
        }))
    }
}

impl<const R: usize, const W: usize> Backend<FourierBackend<R, W>> {
    pub fn prepare_input_msg(
        &mut self,
        f: impl FnOnce(FourierSendRecv) -> std::io::Result<()>,
    ) -> std::io::Result<()> {
        let cvp = if let Some(crate::motor_ctx::CVP {
            position,
            velocity,
            current,
        }) = &self.input_cvp
        {
            let gear_reduction = self.motor_config.gear_reduction;
            crate::motor_ctx::CVP {
                position: position * gear_reduction,
                velocity: velocity * gear_reduction,
                current: *current,
            }
        } else {
            crate::motor_ctx::CVP {
                position: 0.,
                velocity: 0.,
                current: 0.,
            }
        };
        self.backend_specific
            .prepare_input_msg(cvp, &self.motor_config.state, f)
    }

    pub fn parse_cvp(
        &mut self,
        len: usize,
        ring: &mut io_uring::IoUring,
    ) -> Option<crate::motor_ctx::CVP> {
        match self.backend_specific.parse_cvp(len, &self.motor_config) {
            Ok(Some(cvp)) => Some(cvp),
            _ => {
                self.prepare_input_msg(|prep| {
                    prep.queue(&mut ring.submission()).unwrap();
                    ring.submit()?;
                    Ok(())
                })
                .unwrap();
                None
            }
        }
    }

    fn update_input(&mut self, end_tx: &mpsc::Sender<(Ipv4Addr, FourierResponse)>) {
        let next_input = match &self.request_input {
            None => None,
            Some((_, RequestedMotorInput::Cvp(c))) => Some(*c),
            Some((then, RequestedMotorInput::Step(s))) => {
                let now = time::Instant::now();
                let elapsed = now.duration_since(*then);

                if elapsed > s.delay + s.on_dur {
                    let _ = end_tx.send((
                        self.backend_specific.motor.addr(),
                        FourierResponse::EndWaveform,
                    ));
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
                    let _ = end_tx.send((
                        self.backend_specific.motor.addr(),
                        FourierResponse::EndWaveform,
                    ));
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
                    let _ = end_tx.send((
                        self.backend_specific.motor.addr(),
                        FourierResponse::EndWaveform,
                    ));
                }
                cvp
            }
        };

        self.input_cvp = next_input;
    }
}

use std::net::IpAddr;

impl<const R: usize, const W: usize> AsRawFd for FourierBackend<R, W> {
    fn as_raw_fd(&self) -> RawFd {
        self.motor.as_raw_fd()
    }
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum FourierEncodeKind {
    Json,
    Binary,
}

use std::sync::mpsc;

#[derive(Debug)]
pub enum FourierCmd {
    Add(FourierEncodeKind, Option<Timespec>, MotorConfig),
    Remove,
    SetWaveForm(crate::MotorInput),
    StopWaveform,
    Shutdown,
}

use std::time;

#[derive(Debug)]
pub enum FourierResponse {
    OutputCVP(crate::motor_ctx::CVP, time::Instant),
    ControllerAdjustedCVP(crate::motor_ctx::CVP, time::Instant),
    Error(std::io::Error),
    Timeout,
    DuplicateConnections,
    EndWaveform,
}

pub fn event_loop(
    cmd_rx: mpsc::Receiver<(Ipv4Addr, FourierCmd)>,
    err_tx: mpsc::Sender<(Ipv4Addr, FourierResponse)>,
) -> std::io::Result<()> {
    let mut ring = io_uring::IoUring::new(16)?;
    use std::collections::HashMap;
    let mut connections = HashMap::new();

    let mut probe = io_uring::register::Probe::new();
    ring.submitter().register_probe(&mut probe)?;

    use io_uring::opcode::{LinkTimeout, Recv, Send};

    if !probe.is_supported(LinkTimeout::CODE)
        || !probe.is_supported(Recv::CODE)
        || !probe.is_supported(Send::CODE)
    {
        panic!("opcodes not supported");
    }

    let mut request_shutdown = false;

    loop {
        while let Some((ip, cmd)) = match cmd_rx.try_recv() {
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
                FourierCmd::Add(encode, timeout, motor_config) => {
                    if connections.contains_key(&ip) {
                        let _ = err_tx.send((ip, FourierResponse::DuplicateConnections));
                        continue;
                    }

                    let mut motor = Backend::new(
                        motor_config,
                        FourierBackend::<2048, 2048>::new(ip, encode, timeout)
                            .map_err(|_| std::io::Error::other("a"))?,
                        None,
                    );

                    motor
                        .prepare_input_msg(|prep| {
                            prep.queue(&mut ring.submission()).unwrap();
                            ring.submit()?;
                            Ok(())
                        })
                        .unwrap();

                    connections.insert(ip, motor);
                }
                FourierCmd::Shutdown => {
                    request_shutdown = true;
                }
                FourierCmd::SetWaveForm(waveform) => {
                    if let Some(motor) = connections.get_mut(&ip) {
                        use crate::MotorInput;
                        match waveform {
                            MotorInput::Constant(c) => {
                                if !motor.backend_specific.remove_next_recv {
                                    let cvp = crate::motor_ctx::CVP {
                                        velocity: c,
                                        position: 0.,
                                        current: 0.,
                                    };

                                    motor.input_cvp = Some(cvp);
                                    motor.request_input =
                                        Some((time::Instant::now(), RequestedMotorInput::Cvp(cvp)))
                                }
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
                FourierCmd::StopWaveform => {
                    if let Some(motor) = connections.get_mut(&ip) {
                        motor.input_cvp = Some(crate::motor_ctx::CVP {
                            velocity: 0.,
                            position: 0.,
                            current: 0.,
                        });
                        let _ = err_tx.send((ip, FourierResponse::EndWaveform));
                    }
                }
                _ => (),
            }
        }

        let mut completed = ring.completion().next();
        while let Some(entry) = completed {
            completed = ring.completion().next();

            let ip = Ipv4Addr::from_bits(entry.user_data() as u32);

            let Some(motor) = connections.get_mut(&ip) else {
                continue;
            };

            let result = entry.result();
            if result >= 0 {
                let len = result as usize;

                let Some(cvp) = motor.parse_cvp(len, &mut ring) else {
                    // motor does not have data available
                    // an entry has already been submitted.
                    continue;
                };

                let should_drop_motor = motor.backend_specific.remove_next_recv;

                if should_drop_motor {
                    connections.remove(&ip);
                    if request_shutdown && connections.is_empty() {
                        // no more in flight requests and all motors have been shut down
                        return Ok(());
                    }
                    // motor has been idled
                    continue;
                }

                // updates `input_cvp` from `request_input`
                motor.update_input(&err_tx);

                if let Some(controller) = &mut motor.motor_config.controller {
                    let input = motor.input_cvp.unwrap_or_default();

                    let new_input = controller.update(input, cvp, &motor.motor_config.state);
                    let _ = err_tx.send((
                        ip,
                        FourierResponse::ControllerAdjustedCVP(new_input, time::Instant::now()),
                    ));
                    motor.input_cvp = Some(new_input);
                } else {
                    let _ =
                        err_tx.send((ip, FourierResponse::OutputCVP(cvp, time::Instant::now())));
                }

                motor
                    .prepare_input_msg(|prep| {
                        prep.queue(&mut ring.submission()).unwrap();
                        ring.submit()?;
                        Ok(())
                    })
                    .unwrap();
            } else {
                let errno = -result;

                if entry.user_data() & (1 << 32) != 0 {
                    // write err
                    continue;
                }

                match errno {
                    libc::ECANCELED => (),
                    libc::ETIME => {
                        motor.backend_specific.enabled = false;
                        let _ = err_tx.send((ip, FourierResponse::Timeout));
                        motor
                            .prepare_input_msg(|prep| {
                                prep.queue(&mut ring.submission()).unwrap();
                                ring.submit()?;
                                Ok(())
                            })
                            .unwrap();
                    }
                    _ => {
                        let _ = err_tx.send((
                            ip,
                            FourierResponse::Error(std::io::Error::from_raw_os_error(errno)),
                        ));

                        motor
                            .prepare_input_msg(|prep| {
                                prep.queue(&mut ring.submission()).unwrap();
                                ring.submit()?;
                                Ok(())
                            })
                            .unwrap();
                    }
                }
            }
        }
    }
}
