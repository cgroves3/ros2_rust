use std::sync::{Arc, Mutex, MutexGuard};

use rosidl_runtime_rs::{Action, GetResultService};

use crate::error::{RclReturnCode, RclrsError, ToResult};
use crate::vendor::action_msgs::msg::GoalStatus;
use crate::rcl_bindings::*;
use crate::GoalUUID;


pub struct ServerGoalHandleHandle {
    rcl_goal_handle_mtx: Mutex<*mut rcl_action_goal_handle_t>
}

impl ServerGoalHandleHandle {
    pub fn new(goal_handle_mtx: Mutex<*mut rcl_action_goal_handle_t>) -> Self {
        Self { 
            rcl_goal_handle_mtx: goal_handle_mtx 
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<*mut rcl_action_goal_handle_t> {
        self.rcl_goal_handle_mtx.lock().unwrap()
    }
}

impl Drop for ServerGoalHandleHandle {
    fn drop(&mut self) {
        let goal_handle = self.lock();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_goal_handle_fini(*goal_handle);
        }
    }
}

pub struct ServerGoalHandle<'a, T>
where
    T: rosidl_runtime_rs::Action,
{
    handle: Arc<ServerGoalHandleHandle>,
    goal: Arc<<T as Action>::Goal>,
    on_terminal_state: &'a dyn Fn(GoalUUID, Arc<<T::GetResult as GetResultService>::Response>) -> Result<(), RclrsError>,
    on_executing: &'a dyn Fn(GoalUUID) -> Result<(), RclrsError>,
    publish_feedback_cb: &'a dyn Fn(T::Feedback) -> Result<(), RclrsError>
}

unsafe impl<T> Send for ServerGoalHandle<'_, T> where T: rosidl_runtime_rs::Action {}

unsafe impl<T> Sync for ServerGoalHandle<'_, T> where T: rosidl_runtime_rs::Action {}

impl<'a, T> ServerGoalHandle<'a, T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(
        handle: Arc<ServerGoalHandleHandle>,
        goal: Arc<<T as Action>::Goal>,
        on_terminal_state: &'a dyn Fn(GoalUUID, Arc<<T::GetResult as GetResultService>::Response>) -> Result<(), RclrsError>,
        on_executing: &'a dyn Fn(GoalUUID) -> Result<(), RclrsError>,
        publish_feedback_cb: &'a dyn Fn(T::Feedback) -> Result<(), RclrsError>,
    ) -> Self {
        Self {
            handle,
            goal: Arc::clone(&goal),
            on_terminal_state,
            on_executing,
            publish_feedback_cb
        }
    }

    pub fn publish_feedback(&self, feedback: T::Feedback) -> Result<(), RclrsError> {
        (self.publish_feedback_cb)(feedback)
    }

    fn get_state(&self) -> Result<i8, RclrsError> {
        let goal_handle = self.handle.lock();
        let state = &mut GoalStatus::STATUS_UNKNOWN;
        unsafe { rcl_action_goal_handle_get_status(*goal_handle, &mut *state) }.ok()?;
        Ok(*state)
    }

    pub fn get_goal(self) -> Arc<T::Goal> {
        self.goal
    }

    pub fn is_canceling(&self) -> bool {
        match self.get_state() {
            Ok(state) => GoalStatus::STATUS_CANCELING == state,
            Err(..) => false
        }
    }

    pub fn is_active(&self) -> bool {
        let handle = self.handle.lock();
        unsafe { rcl_action_goal_handle_is_active(*handle as *const _) }
    }

    pub fn is_executing(&self) -> bool {
        match self.get_state() {
            Ok(state) => GoalStatus::STATUS_EXECUTING == state,
            Err(..) => false
        }
    }

    //TODO: Is `result` needed for these methods?
    pub fn _abort(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe { 
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_ABORT) 
        }
        .ok()?;
        Ok(())
    }

    pub fn _succeed(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_SUCCEED)
        }
        .ok()?;
        Ok(())
    }

    pub fn _cancel_goal(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL)
        }
        .ok()?;
        Ok(())
    }

    pub fn _canceled(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCELED)
        }
        .ok()?;
        Ok(())
    }

    pub fn _execute(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_EXECUTE)
        }
        .ok()?;
        Ok(())
    }

    pub fn try_canceling(&self) -> bool {
        let handle = self.handle.lock();
        let is_cancelable = unsafe { rcl_action_goal_handle_is_cancelable(*handle) };
        if is_cancelable {
            let ret = unsafe { rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL) };
            if ret != RclReturnCode::Ok as i32 {
              return false;
            }
        }

        // Default state is STATUS_UNKNOWN
        match self.get_state() {
            Ok(state) => {
                if state == GoalStatus::STATUS_CANCELING {
                    let ret = unsafe { rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCELED) };
                    return ret == RclReturnCode::Ok as i32
                }
            }
            Err(_) => return false
        }
        
        false
    }
}