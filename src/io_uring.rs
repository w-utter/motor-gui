use std::collections::HashMap;
use std::os::fd::{AsRawFd, RawFd};

use crate::motor_ctx::MotorCtx;

use io_uring::{Builder, IoUring};

pub(crate) struct IoUringCtx<const R: usize, const W: usize> {
    pub ring: IoUring,
    motor_ctx: HashMap<RawFd, MotorCtx<R, W>>,
}

pub struct IoCtxPeek<'a, const R: usize, const W: usize> {
    pub ring: &'a mut IoUring,
    motors: &'a mut HashMap<RawFd, MotorCtx<R, W>>,
}

impl<const R: usize, const W: usize> IoUringCtx<R, W> {
    pub fn new(entries: u32) -> Result<Self, std::io::Error> {
        let ring = IoUring::new(entries)?;
        let motor_ctx = HashMap::new();

        Ok(Self { ring, motor_ctx })
    }

    pub fn from_builder(builder: &Builder, entries: u32) -> Result<Self, std::io::Error> {
        let ring = builder.build(entries)?;
        let motor_ctx = HashMap::new();

        Ok(Self { ring, motor_ctx })
    }

    pub fn add_motor(&mut self, ctx: MotorCtx<R, W>) -> Option<MotorCtx<R, W>> {
        let fd = ctx.as_raw_fd();
        self.motor_ctx.insert(fd, ctx)
    }

    pub fn remove_motor(&mut self, fd: &RawFd) -> Option<MotorCtx<R, W>> {
        self.motor_ctx.remove(fd)
    }

    fn peek_ctx(&mut self) -> IoCtxPeek<'_, R, W> {
        IoCtxPeek {
            ring: &mut self.ring,
            motors: &mut self.motor_ctx,
        }
    }

    pub fn step(
        &mut self,
    ) -> (
        &mut IoUring,
        Option<(&mut MotorCtx<R, W>, io_uring::cqueue::Entry)>,
    ) {
        let Some(next) = self.ring.completion().next() else {
            return (&mut self.ring, None);
        };

        let userdata = next.user_data();

        let Some(motor) = self.motor_ctx.get_mut(&(userdata as _)) else {
            return (&mut self.ring, None);
        };

        return (&mut self.ring, Some((motor, next)));
    }
}
