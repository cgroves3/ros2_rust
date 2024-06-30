use std::sync::{Arc, Mutex, MutexGuard};

use rosidl_runtime_rs::{Action, GetResultService, Status, SetResult};

use crate::error::{RclReturnCode, RclrsError, ToResult};
// TODO: these may need to be implemented manually using the rcl_bindings. It's not clear when crate::vendor:: is supposed to be used.
use crate::vendor::action_msgs::msg::GoalStatus;
use crate::{rcl_bindings::*, ActionServerHandle, GoalUUID};

/// Internal handle for the C rcl_action_goal_handle_t used by the ServerGoalHandle
pub struct ServerGoalHandleHandle {
    rcl_goal_handle_mtx: Mutex<*mut rcl_action_goal_handle_t>
}

impl ServerGoalHandleHandle {
    /// Creates a new server goal handle 
    pub fn new(goal_handle_mtx: Mutex<*mut rcl_action_goal_handle_t>) -> Self {
        Self { 
            rcl_goal_handle_mtx: goal_handle_mtx 
        }
    }

    /// Locks and unwraps a new server goal handle 
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

/// The ServerGoalHandle tracked by the ActionServer
pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    handle: Arc<ServerGoalHandleHandle>,
    uuid: GoalUUID,
    pub(crate) action_server_handle_mtx: Arc<Mutex<ActionServerHandle>>,
}

unsafe impl<T> Send for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

unsafe impl<T> Sync for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// The server goal handles terminal states
    pub const TERMINAL_STATES: [i8; 3] = [GoalStatus::STATUS_ABORTED, GoalStatus::STATUS_SUCCEEDED, GoalStatus::STATUS_CANCELED];

    /// Creates a new goal for the server
    pub(crate) fn new(
        handle: Arc<ServerGoalHandleHandle>,
        uuid: GoalUUID,
        action_server_handle_mtx: Arc<Mutex<ActionServerHandle>>,
    ) -> Self {
        Self {
            handle,
            uuid,
            action_server_handle_mtx,
        }
    }

    /// Publish feedback about the progress of a goal
    pub fn publish_feedback(&self, feedback: T::Feedback) -> Result<(), RclrsError> {
        self.action_server_handle_mtx.lock().unwrap().publish_feedback::<T>(feedback)
    }

    /// Gets the current status of the goal as an i8, representing the GoalStatus
    pub(crate) fn get_status(&self) -> Result<i8, RclrsError> {
        let goal_handle = self.handle.lock();
        let mut unknown = GoalStatus::STATUS_UNKNOWN;
        let status = &mut unknown;
        unsafe { rcl_action_goal_handle_get_status(*goal_handle, &mut *status) }.ok()?;
        Ok(*status)
    }

    /// Checks if the current goal status is canceling
    pub fn is_canceling(&self) -> bool {
        match self.get_status() {
            Ok(state) => GoalStatus::STATUS_CANCELING == state,
            Err(..) => false
        }
    }

    /// Checks if the goal handle is still active
    pub fn is_active(&self) -> bool {
        let handle = self.handle.lock();
        unsafe { rcl_action_goal_handle_is_active(*handle as *const _) }
    }

    /// Checks if the goal handle is still executing
    pub fn is_executing(&self) -> bool {
        match self.get_status() {
            Ok(state) => GoalStatus::STATUS_EXECUTING == state,
            Err(..) => false
        }
    }

    /// Indicate that a goal could not be reached and has been aborted.
    /// Only call this if the goal was executing but cannot be completed.
    /// This is a terminal state, no more methods should be called on a goal handle after this is called.
    pub fn abort(&mut self, result: <T as Action>::Result) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe { 
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_ABORT) 
        }
        .ok()?;
        type Response<T> = <<T as Action>::GetResult as GetResultService>::Response;
        let mut result_response = Response::<T>::default();
        result_response.set_status(GoalStatus::STATUS_ABORTED);
        result_response.set_result::<T>(result);
        { self.action_server_handle_mtx.lock().unwrap() }.on_terminal_state(&self.uuid, result_response)?;
        Ok(())
    }

    /// Indicate that a goal has succeeded.
    /// Only call this if the goal is executing and has reached the desired final state.
    /// This is a terminal state, no more methods should be called on a goal handle after this is called.
    pub fn succeed(&mut self, result: <T as Action>::Result) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_SUCCEED)
        }
        .ok()?;
        type Response<T> = <<T as Action>::GetResult as GetResultService>::Response;
        let mut result_response = Response::<T>::default();
        result_response.set_status(GoalStatus::STATUS_SUCCEEDED);
        result_response.set_result::<T>(result);
        { self.action_server_handle_mtx.lock().unwrap() }.on_terminal_state(&self.uuid, result_response)?;
        Ok(())
    }

    /// Indicate that a goal has been canceled.
    /// Only call this if the goal is canceling.
    /// This is a terminal state, no more methods should be called on a goal handle after this is called.
    pub fn canceled(&mut self, result: <T as Action>::Result) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCELED)
        }
        .ok()?;
        type Response<T> = <<T as Action>::GetResult as GetResultService>::Response;
        let mut result_response = Response::<T>::default();
        result_response.set_status(GoalStatus::STATUS_CANCELED);
        result_response.set_result::<T>(result);
        { self.action_server_handle_mtx.lock().unwrap() }.on_terminal_state(&self.uuid, result_response)?;
        Ok(())
    }

    /// Marks the goal as executing
    pub fn execute(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_EXECUTE)
        }
        .ok()?;
        Ok(())
    }

    /// Marks the goal as cancel goal
    pub fn cancel_goal(&self) -> Result<(), RclrsError> {
        let handle = self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL)
        }
        .ok()?;
        Ok(())
    }

    /// Tries to cancel the goal handle
    pub(crate) fn try_canceling(&self) -> bool {
        let handle = self.handle.lock();
        let is_cancelable = unsafe { rcl_action_goal_handle_is_cancelable(*handle) };
        if is_cancelable {
            let ret = unsafe { rcl_action_update_goal_state(*handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL) };
            if ret != RclReturnCode::Ok as i32 {
              return false;
            }
        }

        // Default state is STATUS_UNKNOWN
        match self.get_status() {
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