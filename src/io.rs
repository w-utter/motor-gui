use crate::{io_uring::IoUringCtx, motor_backend::MotorBackendConnection, motor_ctx::MotorCtx};
use std::os::fd::{RawFd, AsRawFd};
use std::collections::HashMap;

pub struct IoCtx<const R: usize, const W: usize> {
    backend_map: HashMap<MotorBackendConnection, RawFd>,
    io_uring: IoUringCtx<R, W>,
}

/*
impl <const R: usize, const W: usize> IoCtx<R, W> {
    pub fn new(entries: u32) -> Result<Self, std::io::Error> {
        let io_uring = IoUringCtx::new(entries)?;
        let backend_map = HashMap::new();

        Ok(Self {
            io_uring,
            backend_map,
        })
    }

    pub fn from_builder(builder: &io_uring::Builder, entries: u32) -> Result<Self, std::io::Error> {
        let io_uring = IoUringCtx::from_builder(builder, entries)?;
        let backend_map = HashMap::new();

        Ok(Self {
            io_uring,
            backend_map,
        })
    }

    pub fn insert(&mut self, motor_ctx: MotorCtx<R, W>) -> Result<Option<MotorCtx<R, W>>, MotorBackendConnection> {
        let conn = motor_ctx.backend.connection();
        let fd = motor_ctx.as_raw_fd();

        self.backend_map.insert(conn.clone(), fd).ok_or(conn)?;
        Ok(self.io_uring.add_motor(motor_ctx))
    }

    pub fn remove(&mut self, conn: &MotorBackendConnection) -> Option<MotorCtx<R, W>> {
        let fd = self.backend_map.remove(&conn)?;
        let mut motor = self.io_uring.remove_motor(&fd)?;

        let mut submission = self.io_uring.ring.submission();
        let idle = motor.backend.prepare_idle_msg(&motor.config.state);
        let _ = idle.queue(&mut submission);
        Some(motor)
    }


    pub fn step(
        &mut self,
    ) -> (
        &mut io_uring::IoUring,
        Option<(&mut MotorCtx<R, W>, io_uring::cqueue::Entry)>,
    ) {
        self.io_uring.step()
    }
}
*/
