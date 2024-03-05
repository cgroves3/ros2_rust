use std::sync::{Arc, Mutex, MutexGuard};

use crate::{rcl_bindings::*};
use crate::error::{RclrsError, ToResult};

pub struct ServerGoalHandle<T>
    where
        T: rosidl_runtime_rs::Action,
{
    rcl_handle_mtx: Arc<Mutex<rcl_action_goal_handle_t>>,
    goal_request: Arc<T>,
    on_terminal_state: Box<dyn Fn(GoalUUID, T::Result) -> Result<(), RclrsError>>,
    on_executing: Box<dyn Fn(GoalUUID) -> Result<(), RclrsError>>,
    publish_feedback_cb: Box<dyn Fn(T::Feedback) ->  Result<(), RclrsError>>,
    _marker: PhantomData<T>,
}

impl<T> ServerGoalHandle<T>
    where
        T: rosidl_runtime_rs::Action,
{
    pub fn new(
        rcl_handle: Arc<rcl_action_goal_handle_t>,
        goal_request: Arc<T>,
        on_terminal_state: Box<Fn<GoalUUID, T::Result>>,
        on_executing: Box<Fn<GoalUUID>>,
        publish_feedback_cb: Box<Fn<T::Feedback>>,
    ) -> Self {
        Self {
            rcl_handle_mtx: rcl_handle,
            goal_request: Arc::clone(&goal_request),
            on_terminal_state,
            on_executing,
            publish_feedback_cb,
            _marker: Default::default(),
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_handle_t> {
        self.rcl_handle_mtx.lock().unwrap()
    }

    pub fn publish_feedback(&self, feedback: &T::Feedback) -> () {
        (self.publish_feedback_cb)(feedback);
    }

    pub fn get_goal(&self) -> Arc<T> { self.goal_request }

    pub fn is_canceling(&self) -> bool {
        false
    }

    pub fn is_active(&self) -> bool {
        false
    }

    pub fn is_executing(&self) -> bool {
        false
    }

    //TODO: Is `result` needed for these methods?
    pub fn abort(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = self.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_ABORT)
        }.ok()?;
    }

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = self.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_SUCCEED)
        }.ok()?;
    }

    pub fn cancel_goal(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = self.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL)
        }.ok()?;
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = self.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_CANCELED)
        }.ok()?;
    }
}

impl Drop for ServerGoalHandle {
    fn drop(&mut self) {
        let goal_handle = &mut *self.rcl_handle_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_goal_handle_fini(goal_handle);
        }
    }
}