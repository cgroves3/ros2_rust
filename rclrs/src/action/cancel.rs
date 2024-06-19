use std::sync::{Mutex, MutexGuard};
use crate::{rcl_bindings::*, RclrsError};


/// The cancel request handle
pub struct CancelRequestHandle {
    rcl_action_cancel_request_mtx: Mutex<rcl_action_cancel_request_t>,
}

impl CancelRequestHandle {
    /// Creates a new cancel request handle
    pub fn new() -> Self {
        // SAFETY: Getting a zero-initialized value is always safe.
        let cancel_request = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        Self {
            rcl_action_cancel_request_mtx: Mutex::new(cancel_request),
        }
    }

    /// Locks and unwraps the cancel request handle
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_request_t> {
        self.rcl_action_cancel_request_mtx.lock().unwrap()
    }
}

/// The cancel response handle
pub struct CancelResponseHandle {
    rcl_action_cancel_response_mtx: Mutex<rcl_action_cancel_response_t>,
}
impl CancelResponseHandle {
    /// Creates a new cancel response handle
    pub fn new() -> Self {
        // SAFETY: Getting a zero-initialized value is always safe.
        let rcl_action_cancel_response = unsafe { rcl_action_get_zero_initialized_cancel_response() };
        Self {
            rcl_action_cancel_response_mtx: Mutex::new(rcl_action_cancel_response),
        }
    }

    /// Locks and unwraps the cancel response handle
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_response_t> {
        self.rcl_action_cancel_response_mtx.lock().unwrap()
    }
}

impl Drop for CancelResponseHandle {
    fn drop(&mut self) {
        // SAFETY: No preconditions for this function
        unsafe {
            rcl_action_cancel_response_fini(&mut *self.rcl_action_cancel_response_mtx.get_mut().unwrap());
        }
    }
}