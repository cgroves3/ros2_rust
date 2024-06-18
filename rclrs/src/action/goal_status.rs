use std::sync::{Arc, Mutex};
use crate::rcl_bindings::*;

/// Handle the goal status
pub struct GoalStatus {
    rcl_action_goal_status_mtx: Arc<Mutex<rcl_action_goal_status_t>>
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
pub struct GoalStatusArray {
    rcl_action_goal_status_mtx: Arc<Mutex<rcl_action_goal_status_array_t>>
}

impl GoalStatusArray {
    pub fn new(rcl_action_goal_status_array: rcl_action_goal_status_array_t) -> Self {
        Self {
            rcl_action_goal_status_mtx: Arc::new(Mutex::new(rcl_action_goal_status_array))
        }
    }

    /// Locks and unwraps the cancel response handle
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_status_array_t> {
        self.rcl_action_goal_status_mtx.lock().unwrap()
    }
}

impl Drop for GoalStatusArray {
    fn drop(&mut self) {
        unsafe { 
            rcl_action_goal_status_array_fini(&mut *self.rcl_action_goal_status_mtx.get_mut().unwrap()); 
        }
    }
}