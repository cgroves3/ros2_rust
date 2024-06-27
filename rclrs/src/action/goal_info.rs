use std::sync::{Arc, Mutex, MutexGuard};
use crate::rcl_bindings::*;

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

    // /// Locks and unwraps the goal info
    // pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_info_t> {
    //     self.rcl_action_goal_info_mtx.lock().unwrap()
    // }
}

