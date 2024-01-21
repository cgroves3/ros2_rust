use crate::{rcl_bindings::*, RclrsError};
use std::sync::{Arc, Mutex, MutexGuard};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

use std::marker::PhantomData;

pub type GoalUUID = [u8; RCL_ACTION_UUID_SIZE];

pub enum GoalResponse {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}

pub enum CancelResponse {
    Reject = 1,
    Accept = 2,
}

pub struct ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    _marker: PhantomData<T>,
}

impl<T> ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action client.
    pub(crate) fn new(rcl_node_mtx: Arc<Mutex<rcl_node_t>>, topic: &str) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        Ok(Self {
            _marker: Default::default(),
        })
    }
}


/// Internal struct used by subscriptions.
pub struct ActionServerHandle {
    rcl_action_server_mtx: Mutex<rcl_action_server_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>,
}

impl ActionServerHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_server_t> {
        self.rcl_action_server_mtx.lock().unwrap()
    }
}

impl Drop for ActionServerHandle {
    fn drop(&mut self) {
        let rcl_action_server = self.rcl_action_server_mtx.get_mut().unwrap();
        let rcl_node = &mut *self.rcl_node_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_server_fini(rcl_action_server, rcl_node);
        }
    }
}

pub struct ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub(crate) handle: Arc<ActionServerHandle>,
    handle_goal_cb: fn(&crate::action::GoalUUID, Arc<T::Goal>) -> GoalResponse,
    handle_cancel_cb: fn(Arc<ServerGoalHandle<T>>) -> CancelResponse,
    handle_accepted_cb: fn(Arc<ServerGoalHandle<T>>),
    _marker: PhantomData<T>,
}

impl<T> ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action server.
    pub(crate) fn new(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        topic: &str,
        handle_goal_cb: fn(&crate::action::GoalUUID, Arc<T::Goal>) -> GoalResponse,
        handle_cancel_cb: fn(Arc<ServerGoalHandle<T>>) -> CancelResponse,
        handle_accepted_cb: fn(Arc<ServerGoalHandle<T>>),
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_server = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support_ptr =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: topic.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut action_server_options = unsafe { rcl_action_server_get_default_options() };
        action_server_options.qos = qos.into();
        unsafe {
            // SAFETY: The rcl_action_server is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_action_server_init(
                &mut rcl_action_server,
                &*rcl_node_mtx.lock().unwrap(),
                type_support_ptr,
                topic_c_string.as_ptr(),
                &action_server_options,
            )
                .ok()?;
        }

        let handle = Arc::new(ActionServerHandle{
            rcl_action_server_mtx: Mutex::new(rcl_action_server),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false))
        });

        Ok(Self {
            handle,
            handle_goal_cb,
            handle_cancel_cb,
            handle_accepted_cb,
            _marker: Default::default(),
        })
    }

    pub fn publish_feedback<'a, M: MessageCow<'a, T>>(&self, message: M) -> Result<(), RclrsError> {
        let rmw_message = T::into_rmw_message(message.into_cow());
        let rcl_action_server = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_acttion_publish_feedback(
                rcl_action_server,
                rmw_message.as_ref() as *const <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
            )
                .ok()
        }
    }
}

/// Trait to be implemented by concrete Action Server structs.
///
/// See [`ActionServer<T>`] for an example
pub trait ActionServerBase: Send + Sync {
    /// Internal function to get a reference to the `rcl` handle.
    fn handle(&self) -> &ActionServerHandle;
    /// Tries to take a new request and run the callback with it.
    fn execute(&self) -> Result<(), RclrsError>;
}

impl<T> ActionServerBase for ActionServer<T> {
    fn handle(&self) -> &ActionServerHandle {
        todo!()
    }
    fn execute(&self) -> Result<(), RclrsError> {
        todo!()
    }
}


pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<rcl_action_goal_handle_t>,
    goal_request: Arc<T>,
    _marker: PhantomData<T>,
}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(rcl_handle: Arc<rcl_action_goal_handle_t>,  goal_request: Arc<T>) -> Self {
        Self {
            rcl_handle,
            goal_request: Arc::clone(&goal_request),
            _marker: Default::default(),
        }
    }

    pub fn publish_feedback(&self, feedback: &T::Feedback) -> () {
        let feedback_copy = feedback.clone();
        unsafe {
            rcl_action_publish_feedback(

            )
        }
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

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        Ok(())
    }
}
