pub(crate) mod fourier;

use fourier::{FourierBackend, FourierSendRecv};

use io_uring::squeue::Entry as SQEntry;
use io_uring::squeue::PushError as SQPushError;
use io_uring::types::Timespec;

use crate::motor_ctx::{ControlState, MotorConfig};
use std::os::fd::{AsRawFd, RawFd};
use std::net::IpAddr;

#[derive(Hash, PartialEq, Eq, Clone)]
pub(crate) enum MotorBackendConnection {
    Ip(IpAddr, Option<u16>),
}

pub(crate) enum MotorBackend<const R: usize, const W: usize> {
    Fourier(FourierBackend<R, W>),
}

impl<const R: usize, const W: usize> AsRawFd for MotorBackend<R, W> {
    fn as_raw_fd(&self) -> RawFd {
        match self {
            Self::Fourier(f) => f.as_raw_fd(),
        }
    }
}

pub enum MotorBackendParseError {
    Fourier(amber_aios::Err),
}

/*
impl<const R: usize, const W: usize> MotorBackend<R, W> {
    pub fn connection(&self) -> MotorBackendConnection {
        match self {
            Self::Fourier(f) => MotorBackendConnection::Ip(f.ip_addr(), None)
        }
    }

    pub fn prepare_input_msg(
        &mut self,
        cvp: crate::motor_ctx::CVP,
        mode: &ControlState,
    ) -> MotorBackendIoPrep {
        match self {
            Self::Fourier(f) => MotorBackendIoPrep::Fourier(f.prepare_input_msg(cvp, mode)),
        }
    }

    pub fn prepare_idle_msg(
        &mut self,
        mode: &ControlState,
    ) -> MotorBackendIoPrep {
        match self {
            Self::Fourier(f) => MotorBackendIoPrep::Fourier(f.prepare_idle_msg(mode)),
        }
    }

    pub fn parse_cvp(
        &mut self,
        len: usize,
        config: &MotorConfig,
    ) -> Result<Option<crate::motor_ctx::CVP>, MotorBackendParseError> {
        match self {
            Self::Fourier(f) => Ok(f.parse_cvp(len, config)?),
        }
    }
}
*/

pub(crate) enum MotorBackendIoPrep {
    Fourier(FourierSendRecv),
}

impl MotorBackendIoPrep {
    pub fn queue(&self, q: &mut io_uring::SubmissionQueue<'_>) -> Result<(), SQPushError> {
        match self {
            Self::Fourier(f) => f.queue(q),
        }
    }
}
