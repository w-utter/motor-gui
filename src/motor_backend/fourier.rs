use io_uring::squeue::Entry as SQEntry;
use io_uring::squeue::PushError as SQPushError;
use io_uring::types::Timespec;

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
}

impl MotorUiConfig {
    pub(crate) fn display(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;

        let Self {
            ip_buf_storage,
            ip,
            encoding,
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
}

impl Default for MotorUiConfig {
    fn default() -> Self {
        Self {
            ip_buf_storage: String::new(),
            ip: None,
            encoding: FourierEncodeKind::Binary,
        }
    }
}

const DEFAULT_FOURIER_READ_TIMEOUT: Timespec = Timespec::new().nsec(100000000);

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

pub struct Backend<T> {
    control_state: ControlState,
    motor_config: MotorConfig,
    input_cvp: Option<crate::motor_ctx::CVP>,
    backend_specific: T,
}

impl <T> Backend<T> {
    fn new(
        control_state: ControlState,
        motor_config: MotorConfig,
        backend_specific: T,
    ) -> Self {
        Self {
            control_state,
            motor_config,
            input_cvp: None,
            backend_specific,
        }
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
    pub fn new(ip_addr: Ipv4Addr, encode: FourierEncodeKind, read_timeout: Option<Timespec>) -> Result<Self, amber_aios::Err> {
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
    ) -> FourierSendRecv {
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
            self.prepare_msg_from_cmd(&enable_cmd)
        } else {
            match (mode, &self.config.encode) {
                (ControlState::Position { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_pos_json)
                }
                (ControlState::Position { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_pos)
                }
                (ControlState::Velocity { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_vel_json)
                }
                (ControlState::Velocity { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_vel)
                }
                (ControlState::Torque { .. }, FourierEncodeKind::Json) => {
                    self.prepare_msg_from_cmd(&set_i_json)
                }
                (ControlState::Torque { .. }, FourierEncodeKind::Binary) => {
                    self.prepare_msg_from_cmd(&set_i)
                }
            }
        }
    }

    pub fn prepare_idle_msg(
        &mut self,
        mode: &ControlState,
    ) -> FourierSendRecv {
        self.prepare_input_msg(crate::motor_ctx::CVP { position: 0., velocity: 0., current: 0. }, mode)
    }

    pub fn prepare_msg_from_cmd<'a, 'b, C: amber_aios::cmds::SerializableCommand<'a>>(
        &'b mut self,
        cmd: &'b C,
    ) -> FourierSendRecv {
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
                .user_data(motor.addr().to_bits() as u64)
        };

        let recv = {
            let recv_buf = motor.read_buf_mut();

            Recv::new(Fd(fd), recv_buf.as_mut_ptr(), recv_buf.len() as _)
                .build()
                .flags(Flags::IO_LINK)
                .user_data((fd as u64))
        };

        let read_timeout = self
            .config
            .read_timeout
            .unwrap_or(DEFAULT_FOURIER_READ_TIMEOUT);

        let recv_timeout = LinkTimeout::new(&read_timeout).build().user_data(fd as _);

        FourierSendRecv {
            send,
            recv,
            recv_timeout,
        }
    }

    pub fn parse_cvp(
        &mut self,
        len: usize,
        config: &MotorConfig,
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
            position: position / config.gear_reduction,
            velocity: velocity / config.gear_reduction,
            current,
        }))
    }

    pub fn ip_addr(&self) -> IpAddr {
        IpAddr::V4(self.motor.addr())
    }
}

impl <const R: usize, const W: usize> Backend<FourierBackend<R, W>> {
    pub fn prepare_input_msg(
        &mut self,
    ) -> FourierSendRecv {

        let cvp = if let Some(crate::motor_ctx::CVP {position, velocity, current}) = &self.input_cvp {
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
        self.backend_specific.prepare_input_msg(cvp, &self.control_state)
    }

    pub fn prepare_idle_msg(
        &mut self,
    ) -> FourierSendRecv {
        self.backend_specific.prepare_idle_msg(&self.control_state)
    }

    pub fn parse_cvp(
        &mut self,
        len: usize,
        ring: &mut io_uring::IoUring,
    ) -> Option<crate::motor_ctx::CVP> {
        match self.backend_specific.parse_cvp(len, &self.motor_config) {
            Ok(Some(cvp)) => Some(cvp),
            _ => {
                let prep = self.prepare_input_msg();
                prep.queue(&mut ring.submission());
                ring.submit();
                None
            }
        }
    }
}

use std::net::IpAddr;

impl<const R: usize, const W: usize> AsRawFd for FourierBackend<R, W> {
    fn as_raw_fd(&self) -> RawFd {
        self.motor.as_raw_fd()
    }
}

use super::MotorBackendParseError;
impl From<amber_aios::Err> for MotorBackendParseError {
    fn from(f: amber_aios::Err) -> MotorBackendParseError {
        MotorBackendParseError::Fourier(f)
    }
}

#[derive(PartialEq, Eq)]
pub enum FourierEncodeKind {
    Json,
    Binary,
}

use std::sync::mpsc;

enum FourierCmd {
    Add(FourierEncodeKind, Option<Timespec>, ControlState, MotorConfig),
    Remove,
    SetCVP(crate::motor_ctx::CVP),
    Shutdown,
}

enum FourierResponse {
    OutputCVP(crate::motor_ctx::CVP),
    ControllerAdjustedCVP(crate::motor_ctx::CVP),
    Error(std::io::Error),
    DuplicateConnections,
}

fn event_loop(cmd_rx: mpsc::Receiver<(Ipv4Addr, FourierCmd)>, err_tx: mpsc::Sender<(Ipv4Addr, FourierResponse)>) -> std::io::Result<()> {
    let mut ring = io_uring::IoUring::new(16)?;
    use std::collections::HashMap;
    let mut connections = HashMap::new();

    let mut request_shutdown = false;

    loop {
        if !request_shutdown {
            while let Some((ip, cmd)) = match cmd_rx.try_recv() {
                Ok(cmd) => Some(cmd),
                Err(mpsc::TryRecvError::Disconnected) => return Err(std::io::Error::new(std::io::ErrorKind::Other, "disconnected")),
                _ => None,
            } {
                match cmd {
                    FourierCmd::Add(encode, timeout, ctrl_state, motor_config) => {
                        let mut motor = Backend::new(ctrl_state, motor_config, FourierBackend::<1024, 1024>::new(ip, encode, timeout).map_err(|_| std::io::Error::other("a"))?);

                        if connections.contains_key(&ip) {
                            err_tx.send((ip, FourierResponse::DuplicateConnections));
                            continue;
                        }

                        let prep = motor.prepare_input_msg();

                        prep.queue(&mut ring.submission());
                        ring.submit()?;

                        connections.insert(ip, motor);
                    }
                    FourierCmd::Remove => {
                        if let Some(motor) = connections.get_mut(&ip) {
                            motor.backend_specific.remove_next_recv =  true;
                            motor.input_cvp = None;
                        }
                    }
                    FourierCmd::Shutdown => {
                        for motor in connections.values_mut() {
                            motor.backend_specific.remove_next_recv =  true;
                            motor.input_cvp = None;
                        }
                        request_shutdown = true;
                    }
                    FourierCmd::SetCVP(cvp) => {
                        if let Some(motor) = connections.get_mut(&ip) {
                            if !motor.backend_specific.remove_next_recv {
                                motor.input_cvp = Some(cvp);
                            }
                        }
                    }
                }
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
                use amber_aios::cmds::SerializableCommand;
                let len = result as usize;

                let Some(cvp) = motor.parse_cvp(len, &mut ring) else {
                    // motor does not have data available
                    // an entry has already been submitted.
                    continue;
                };

                err_tx.send((ip, FourierResponse::OutputCVP(cvp)));

                let should_drop_motor = motor.backend_specific.remove_next_recv;

                if should_drop_motor {
                    connections.remove(&ip);
                    if request_shutdown && connections.is_empty() {
                        // no more in flight requests and all motors have been shut down
                        return Ok(())
                    }
                    // motor has been idled
                    continue;
                }

                if let Some(controller) = &mut motor.motor_config.controller {
                    let input = motor.input_cvp.unwrap_or_default();

                    let new_input = controller.update(input, cvp, &motor.control_state);
                    err_tx.send((ip, FourierResponse::ControllerAdjustedCVP(new_input)));
                    motor.input_cvp = Some(new_input);
                }

                let prep = motor.prepare_input_msg();
                prep.queue(&mut ring.submission());
                ring.submit();
            } else {
                let errno = -result;
                match errno {
                    libc::ECANCELED => (),
                    libc::ETIME => {
                        motor.backend_specific.enabled = false;

                        let prep = motor.prepare_input_msg();
                        prep.queue(&mut ring.submission());
                        ring.submit();
                    }
                    _ => {
                        err_tx.send((ip, FourierResponse::Error(std::io::Error::from_raw_os_error(errno))));

                        let prep = motor.prepare_input_msg();
                        prep.queue(&mut ring.submission());
                        ring.submit();
                    }
                }
            }
        }
    }
    Ok(())
}
