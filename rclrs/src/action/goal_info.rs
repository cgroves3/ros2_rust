use std::sync::{Arc, Mutex};
use crate::rcl_bindings::*;

pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Arc<Mutex<rcl_action_goal_info_t>>
}

impl GoalInfoHandle {
    fn new(goal_info_mtx: rcl_action_goal_info_t) {
        Self {
            rcl_action_goal_info_mtx: goal_info_mtx
        }
    }
}