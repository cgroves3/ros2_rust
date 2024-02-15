use crate::{rcl_bindings::*, RclrsError};
use std::sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard};
use std::collections::HashMap;

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


/// Internal struct used by action servers.
pub struct ActionServerHandle {
    rcl_action_server_mtx: Mutex<rcl_action_server_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
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
    pub(crate) goal_handles: Arc<HashMap<crate::action::GoalUUID, Arc<ServerGoalHandle<>>>>,
    pub(crate) handle: Arc<ActionServerHandle>,
    handle_goal_cb: fn(&crate::action::GoalUUID, Arc<T::Goal>) -> GoalResponse,
    handle_cancel_cb: fn(Arc<ServerGoalHandle<T>>) -> CancelResponse,
    handle_accepted_cb: fn(Arc<ServerGoalHandle<T>>),
    goal_request_ready: Arc<AtomicBool>,
    cancel_request_ready: Arc<AtomicBool>,
    result_request_ready: Arc<AtomicBool>,
    goal_expired: Arc<AtomicBool>,
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
            ).ok()?;
        }

        let handle = Arc::new(ActionServerHandle{
            rcl_action_server_mtx: Mutex::new(rcl_action_server),
            rcl_node_mtx,
        });

        Ok(Self {
            goal_handles: HashMap::new(),
            handle,
            handle_goal_cb,
            handle_cancel_cb,
            handle_accepted_cb,
            goal_request_ready: Arc::new(AtomicBool::new(false)),
            cancel_request_ready: Arc::new(AtomicBool::new(false)),
            result_request_ready: Arc::new(AtomicBool::new(false)),
            goal_expired: Arc::new(AtomicBool::new(false)),
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
            rcl_action_publish_feedback(
                rcl_action_server,
                rmw_message.as_ref() as *const <T as Message>::RmwMsg as *mut _,
                std::ptr::null_mut(),
            ).ok()?;
        }
    }

    pub fn take_goal_request(&self) -> Result<(T::Goal, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
        <<T as rosidl_runtime_rs::Action>::Goal as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_goal_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            ).ok()?
        }
        Ok((T::Goal::from_rmw_message(&request_out) , request_id_out));
    }

    pub fn take_result_request(&self) -> Result<(T::Impl::GetResultService::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
        <<T as rosidl_runtime_rs::Action>::Impl::GetResultService::Request as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_result_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            ).ok()?
        }
        Ok((T::Impl::GetResultService::Request::from_rmw_message(&request_out) , request_id_out));
    }

    pub fn take_cancel_request(&self) -> Result<(action_msgs::srv::CancelGoal::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
        <action_msgs::srv::CancelGoal::Request as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_cancel_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            ).ok()?
        }
        Ok((action_msgs::srv::CancelGoal::Request::from_rmw_message(&request_out) , request_id_out));
    }

    pub fn execute_goal_request_received(&self) -> Result<(), RclrsError> {
        let (goal_request, mut req_id) = match self.take_goal_request() {
            Ok((req, req_id)) => (req, req_id),
            Err(RclrsError::RclActionError {
                    code: RclActionReturnCode::ActionServerTakeFailed,
                    ..
                }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // server was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };

        let res = (*self.handle_goal_cb.lock().unwrap())(&req_id, goal_request);
        if (res == GoalResponse::AcceptAndExecute || res == GoalResponse::AcceptAndDefer) {
            let goal_info_handle = GoalInfoHandle::new(self.handle);
            let uuid = goal_request.uuid;
            goal_info_handle.lock().goal_id.uuid = uuid;
            let (new_goal_handle, error) = self.accept_new_goal(goal_info_handle, goal_request);
            let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());
            let handle = &*self.handle.lock();
            unsafe {
                // SAFETY: The response type is guaranteed to match the service type by the type system.
                rcl_action_send_goal_response(
                    handle,
                    &mut req_id,
                    rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
                )
            }.ok()?;
            { self.goal_handles.lock().unwrap() }.insert(uuid, new_goal_handle);
            if (res == GoalResponse::AcceptAndExecute) {
                unsafe { rcl_action_update_goal_state(new_goal_handle, GOAL_EVENT_EXECUTE); }.ok()?;
            }
            self.publish_status()?;
            self.call_goal_accepted_cb(new_goal_handle, uuid, goal_request);
        }
    }

    // This might not be needed if the server_goal_handle is prepared with the right args and already added to goal_handles
    pub fn call_goal_accepted_cb(&self, server_goal_handle: ServerGoalHandle, goal_uuid: GoalUUID, goal_request: T::Goal) -> Result<(), RclrsError> {

    }

    pub fn execute_cancel_request_received(&self) -> Result<(), RclrsError>  {
        let (cancel_request, mut req_id) = match self.take_cancel_request() {
            Ok((req, req_id)) => (req, req_id),
            Err(RclrsError::RclActionError {
                    code: RclActionReturnCode::ActionServerTakeFailed,
                    ..
                }) => {
                // Spurious wakeup – this may happen even when a waitset indicated that this
                // server was ready, so it shouldn't be an error.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let res = (*self.handle_cancel_cb.lock().unwrap())(&req_id, cancel_request);
        let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_action_send_cancel_response(
                handle,
                &mut req_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        }.ok()
    }
    pub fn execute_result_request_received(&self) -> Result<(), RclrsError> {

    }
    pub fn execute_check_expired_goals(&self) -> Result<(), RclrsError> {

    }

    pub fn publish_status(&self) -> Result<(), RclrsError> {
        let mut num_goals: usize = 0;
        let mut goal_handles = std::ptr::null_mut();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_server_get_goal_handles(
                handle,
                goal_handles,
                num_goals
            )
        }.ok()?;
        let goal_status_array = Arc::new(action_msgs::msg::GoalStatusArray::default());
        goal_status_array.reserve(num_goals);
        let mut status_array = unsafe { rcl_action_get_zero_initialized_goal_status_array() };
        unsafe {
            rcl_action_get_goal_status_array(
                handle,
                &mut status_array
            )
        }.ok()?;
        unsafe { rcl_action_goal_status_array_fini(&mut status_array) }.ok()?;
        for i in 0..status_array.msg.status_list.size {
            let status_msg = status_array.msg.status_list[i];

            let goal_status = action_msgs::msg::GoalStatus::default();
            goal_status.status = status_msg.status;
            goal_status.goal_info.stamp = status_msg.stamp;
            goal_status.uuid = status_msg.goal_info.uuid;
            goal_status_array.status_list.append(goal_status);
        }
        unsafe {
            rcl_action_publish_status(handle, goal_status_array)
        }.ok()?;
    }

    pub fn accept_new_goal(&self, goal_info: GoalInfoHandle, goal_req: Arc<T>) -> Result<ServerGoalHandle<T>, RclrsError> {
        let handle = &*self.handle.lock();
        let goal_info_handle = &goal_info.lock();
        let goal_handle = unsafe { rcl_action_accept_new_goal(handle, goal_info_handle).ok()? };
        //TODO: Fix this to include to new args
        Ok(ServerGoalHandle::new(Arc::new(goal_handle), goal_req));
    }
}

pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>,
    rcl_action_server: Arc<ActionServerHandle>,
}
impl GoalInfoHandle {
    pub fn new(rcl_action_server_mtx: Arc<ActionServerHandle>) -> Self {
        let mut goal_info = unsafe { rcl_action_get_zero_initialized_goal_info };
        Self {
            rcl_action_goal_info_mtx: Mutex::new(goal_info),
            rcl_action_server
        }
    }
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_info_t> {
        self.rcl_action_goal_info_mtx.lock().unwrap()
    }
}

impl Drop for GoalInfoHandle {
    fn drop(&mut self) {
        let goal_info = &mut *self.rcl_action_goal_info_mtx.lock().unwrap();
        let rcl_action_server = &self.rcl_action_server.lock();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_goal_info_fini(goal_info, rcl_action_server);
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

    /// Sets the goal request ready state to value
    fn set_goal_request_ready(&self, value: bool) -> ();
    /// Sets the cancel request ready state to value
    fn set_cancel_request_ready(&self, value: bool) -> ();
    /// Sets the result request ready state to value
    fn set_result_request_ready(&self, value: bool) -> ();
    /// Sets the goal expired state to value
    fn set_goal_expired(&self, value: bool) -> ();

    fn is_ready(&self) -> bool;
}

impl<T> ActionServerBase for ActionServer<T> {
    fn handle(&self) -> &ActionServerHandle {
        &self.handle
    }
    fn execute(&self) -> Result<(), RclrsError> {
        if (self.goal_request_ready.load(Ordering::SeqCst)) {
            execute_goal_request_received();
        }
        else if (self.cancel_request_ready.load(Ordering::SeqCst)) {
            execute_cancel_request_received();
        }
        else if (self.result_request_ready.load(Ordering::SeqCst)) {
            execute_result_request_received();
        }
        else if (self.goal_expired.load(Ordering::SeqCst)) {
            execute_check_expired_goals();
        }
        else {
            panic!("Executing action server but nothing is ready");
        }
    }
    fn set_goal_request_ready(&self, value: bool) -> () {
        self.goal_request_ready.store(value, Ordering::SeqCst)
    }
    fn set_cancel_request_ready(&self, value: bool) -> () {
        self.cancel_request_ready.store(value, Ordering::SeqCst)
    }
    fn set_result_request_ready(&self, value: bool) -> () {
        self.result_request_ready.store(value, Ordering::SeqCst)
    }
    fn set_goal_expired(&self, value: bool) -> () {
        self.goal_expired.store(value, Ordering::SeqCst)
    }

    fn is_ready(&self) -> bool {
        self.goal_request_ready.load(Ordering::SeqCst) ||
            self.cancel_request_ready.load(Ordering::SeqCst) ||
            self.result_request_ready.load(Ordering::SeqCst) ||
            self.goal_expired.load(Ordering::SeqCst);
    }
}


pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    rcl_handle: Arc<rcl_action_goal_handle_t>,
    goal_request: Arc<T>,
    on_terminal_state: Box<Fn<GoalUUID, T::Result>>,
    on_executing: Box<Fn<GoalUUID>>,
    publish_feedback_cb: Box<Fn<T::Feedback>>,
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
            rcl_handle,
            goal_request: Arc::clone(&goal_request),
            on_terminal_state,
            on_executing,
            publish_feedback_cb,
            _marker: Default::default(),
        }
    }

    pub fn publish_feedback(&self, feedback: &T::Feedback) -> () {

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
