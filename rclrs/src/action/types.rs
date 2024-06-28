use std::sync::{Arc, Mutex, MutexGuard};
use crate::rcl_bindings::*;


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

/// A handle for the goal info
pub struct GoalInfo {
    _rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>
}

impl GoalInfo {
    /// Creates a new goal info
    pub fn new() -> Self {
        let goal_info = unsafe { rcl_action_get_zero_initialized_goal_info() };
        Self {
            _rcl_action_goal_info_mtx: Mutex::new(goal_info)
        }
    }
}

/// Handle the goal status
pub struct GoalStatus {
    _rcl_action_goal_status_mtx: Arc<Mutex<rcl_action_goal_status_t>>
}

impl GoalStatus {
    /// Creates a new goal status handle
    pub fn new(rcl_action_goal_status: rcl_action_goal_status_t) -> Self {
        Self {
            _rcl_action_goal_status_mtx: Arc::new(Mutex::new(rcl_action_goal_status))
        }
    }
}

impl GoalStatus {
    /// Indicates status has not been properly set.
    pub const STATUS_UNKNOWN: i8 = 0;
    /// The goal has been accepted and is awaiting execution.
    pub const STATUS_ACCEPTED: i8 = 1;
    /// The goal is currently being executed by the action server.
    pub const STATUS_EXECUTING: i8 = 2;
    /// The client has requested that the goal be canceled and the action server has
    /// accepted the cancel request.
    pub const STATUS_CANCELING: i8 = 3;
    /// The goal was achieved successfully by the action server.
    pub const STATUS_SUCCEEDED: i8 = 4;
    /// The goal was canceled after an external request from an action client.
    pub const STATUS_CANCELED: i8 = 5;
    /// The goal was terminated by the action server without an external request.
    pub const STATUS_ABORTED: i8 = 6;
}

/// Handle the goal status array
pub struct GoalStatusArrayHandle {
    rcl_action_goal_status_array_mtx: Mutex<rcl_action_goal_status_array_t>
}

impl GoalStatusArrayHandle {
    /// Creates a new goal status array handle
    pub fn new(rcl_action_goal_status_array: rcl_action_goal_status_array_t) -> Self {
        Self {
            rcl_action_goal_status_array_mtx: Mutex::new(rcl_action_goal_status_array)
        }
    }

    /// Locks and unwraps the cancel response handle
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_status_array_t> {
        self.rcl_action_goal_status_array_mtx.lock().unwrap()
    }

    // /// Locks and unwraps the cancel response handle
    // pub(crate) fn get_mut(&self) -> &mut rcl_action_goal_status_array_t {
    //     self.rcl_action_goal_status_array_mtx.get_mut().unwrap()
    // }
}

impl Drop for GoalStatusArrayHandle {
    fn drop(&mut self) {
        unsafe { 
            rcl_action_goal_status_array_fini(&mut *self.rcl_action_goal_status_array_mtx.get_mut().unwrap()); 
        }
    }
}