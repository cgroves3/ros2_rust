mod client;
mod server;
mod types;

pub use client::*;
pub use server::*;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard};

use rosidl_runtime_rs::{Action, GetResultService, Message, Status, SetResult};

use crate::error::{RclReturnCode, RclrsError, ToResult};
use crate::vendor::action_msgs::msg::GoalStatus;
use crate::{rcl_bindings::*, MessageCow};


/// The Goal UUID
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub struct GoalUUID([u8; RCL_ACTION_UUID_SIZE]);

impl GoalUUID {
    /// Creates a new goal uuid
    pub fn new(uuid: [u8; RCL_ACTION_UUID_SIZE]) -> Self {
        Self(uuid)
    }
}


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
    pub(crate) goal_results: Arc<Mutex<HashMap<GoalUUID, <T::GetResult as GetResultService>::Response>>>,
    pub(crate) result_requests: Arc<Mutex<HashMap<GoalUUID, Vec<rmw_request_id_t>>>>
}

unsafe impl<T> Send for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

unsafe impl<T> Sync for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new goal for the server
    pub(crate) fn new(
        handle: Arc<ServerGoalHandleHandle>,
        uuid: GoalUUID,
        action_server_handle_mtx: Arc<Mutex<ActionServerHandle>>,
        goal_results: Arc<Mutex<HashMap<GoalUUID, <T::GetResult as GetResultService>::Response>>>,
        result_requests: Arc<Mutex<HashMap<GoalUUID, Vec<rmw_request_id_t>>>>
    ) -> Self {
        Self {
            handle,
            uuid,
            action_server_handle_mtx,
            goal_results,
            result_requests,
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
        self.on_terminal_state(&self.uuid, result_response)?;
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
        self.on_terminal_state(&self.uuid, result_response)?;
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
        self.on_terminal_state(&self.uuid, result_response)?;
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


    fn on_terminal_state(&self, goal_uuid: &GoalUUID, result_response: <T::GetResult as GetResultService>::Response) -> Result<(), RclrsError> {
        self.publish_result(goal_uuid, result_response)?;
        { self.action_server_handle_mtx.lock().unwrap() }.publish_status()?;
        self.notify_goal_terminal_state()?;
        Ok(())
    } 

    
    /// Publishes the given result for the given goal uuid
    fn publish_result(
        &self,
        goal_uuid: &GoalUUID,
        result: <T::GetResult as GetResultService>::Response,
    ) -> Result<(), RclrsError> 
    where T: rosidl_runtime_rs::Action {
        let goal_info = GoalInfoHandle::new();
        goal_info.lock().goal_id.uuid.copy_from_slice(&goal_uuid.0);
        let unwrapped_handle = self.action_server_handle_mtx.lock().unwrap();
        let server_handle = &*unwrapped_handle.lock();
        let goal_info_handle = &*goal_info.lock();
        let goal_exists = unsafe { rcl_action_server_goal_exists(server_handle, goal_info_handle) };
        if !goal_exists {
            panic!("Asked to publish a result for a goal that does not exist");
        }

        // If there are clients who already asked for the result, send it to them
        if let Some(req_ids) = { self.result_requests.lock().unwrap() }.get_mut(&goal_uuid)
        {
            let result_rmw_message = <<<T as Action>::GetResult as GetResultService>::Response as Message>::into_rmw_message(result.into_cow());
            type RmwMsg<T> = <<<T as Action>::GetResult as GetResultService>::Response as Message>::RmwMsg;
            for req_id in req_ids {
                unsafe {
                    rcl_action_send_result_response(
                        server_handle,
                        req_id as *mut _,
                        result_rmw_message.as_ref() as *const RmwMsg<T> as *mut _,
                    )
                }
                .ok()?;
            }
        } else {
            { self.goal_results.lock().unwrap() }.insert(goal_uuid.clone(), result);
        }
        Ok(())
    }

    /// Notifies that a goal handle reached a terminal state
    fn notify_goal_terminal_state(&self) -> Result<(), RclrsError> {
        let unwrapped_handle = self.action_server_handle_mtx.lock().unwrap();
        let handle = &*unwrapped_handle.lock();
        unsafe { rcl_action_notify_goal_done(handle) }.ok()?;
        Ok(())
    }
}