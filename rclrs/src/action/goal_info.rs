use std::sync::{Arc, Mutex};
use crate::rcl_bindings::*;

/// Handle for the goal info
pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Arc<Mutex<rcl_action_goal_info_t>>
}

impl GoalInfoHandle {
    /// Creates a new goal info
    pub fn new(rcl_action_goal_info: rcl_action_goal_info_t) -> Self {
        Self {
            rcl_action_goal_info_mtx: Arc::new(Mutex::new(rcl_action_goal_info))
        }
    }

    /// Locks and unwraps the goal info
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_info_t> {
        self.rcl_action_goal_info_mtx.lock().unwrap()
    }
}