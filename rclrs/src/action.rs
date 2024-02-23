use std::sync::{atomic::AtomicBool, Arc, Mutex, MutexGuard};
use std::collections::HashMap;

use crate::{rcl_bindings::*};
use crate::error::{RclrsError, ToResult};
use crate::server_goal_handle::{ServerGoalHandle, GoalEvent};

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
    pub(crate) goal_handles: Arc<Mutex<HashMap<crate::action::GoalUUID, Arc<ServerGoalHandle<T>>>>>,
    pub(crate) goal_results: Arc<Mutex<HashMap<crate::action::GoalUUID, T::Result>>>,
    pub(crate) result_requests: Arc<Mutex<HashMap<crate::action::GoalUUID, Vec<rmw_request_id_t>>>>,
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
        let mut rcl_action_server = unsafe { rcl_action_get_zero_initialized_server() };
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
            goal_handles: Arc:new(Mutex::new(HashMap::new())),
            goal_results: Arc:new(Mutex::new(HashMap::new())),
            result_requests: Arc:new(Mutex::new(HashMap::new())),
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
            // TODO: rclcpp also inserts into a map of GoalUUID and rcl_action_goal_handle_t pairs for some reason. Unsure if this is needed
            if (res == GoalResponse::AcceptAndExecute) {
                unsafe { rcl_action_update_goal_state(new_goal_handle, GoalEvent::Execute); }.ok()?;
            }
            self.publish_status()?;
            self.call_goal_accepted_cb(new_goal_handle, uuid, goal_request);
        }
    }

    pub fn call_goal_accepted_cb(&self, goal_handle_t: rcl_action_goal_handle_t, goal_uuid: GoalUUID, goal_request: T::Goal) -> () {
        let on_terminal_state = |uuid: GoalUUID, result: T::Result| -> Result<(), RclrsError> {
            self.publish_result(uuid, result);
            self.publish_status();
            self.notify_goal_terminal_state();
            { self.goal_handles.lock().unwrap() }.remove(uuid);
            Ok(());
        };

        let on_executing = |uuid: GoalUUID| -> Result<(), RclrsError> {
            self.publish_status();
        };

        let publish_feedback = |feedback_msg: T::Feedback| -> Result<(), RclrsError>{
            self.publish_feedback(feedback_msg);
        };
        let goal_handle_arc = Arc::new(ServerGoalHandle::new(Arc::new(Mutex::new(goal_handle_t)), Box::new(on_terminal_state), Box::new(on_executing), Box::new(publish_feedback), goal_req));
        { self.goal_handles.lock().unwrap() }.insert(uuid, goal_handle_arc);
        (self.handle_accepted_cb)(goal_handle_arc);
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
        //TODO: Set the goal uuid and timestamp for the cancel request
        let cancel_request_handle = CancelRequestHandle::new(cancel_request.goal_info);
        let cancel_response_handle = CancelResponseHandle::new();
        let handle = &*self.handle.lock();
        let request_handle = cancel_request_handle.lock().unwrap();
        let response_handle = cancel_response_handle.lock().unwrap();
        unsafe {
            rcl_action_process_cancel_request(
                handle,
                &request_handle,
                &mut cancel_response_handle
            )
        }.ok()?;
        let response = Arc::new(action_msgs::srv::CancelGoal::Response::default());
        response.return_code = response_handle.msg.return_code;
        let goals = response_handle.msg.goals_canceling;

        for i in 0..goal.size {
            let goal_info = goals.data[i];
            let uuid = goal_info.uuid;
            let cb_response_code = self.call_handle_cancel_callback(uuid);
            if (CancelResponse::Accept == cb_response_code) {
                let rs_goal_info = action_msgs::msgs::GoalInfo::default();
                rs_goal_info.goal_id.uuid = goal_info.uuid;
                //TODO: Set the timestamp for the rs_goal_info
                response.goals_canceling.push(rs_goal_info);
            }
        }

        let res = (*self.handle_cancel_cb.lock().unwrap())(&req_id, cancel_request);
        let rmw_message = <T::Response as Message>::into_rmw_message(res.into_cow());

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

    pub fn accept_new_goal(&self, goal_info: GoalInfoHandle, goal_req: Arc<T>) -> Result<rcl_action_goal_handle_t, RclrsError> {
        let handle = &*self.handle.lock();
        let goal_info_handle = &goal_info.lock();
        let goal_handle = unsafe { rcl_action_accept_new_goal(handle, goal_info_handle).ok()? };
    }

    pub fn publish_result(&self, goal_uuid: GoalUUID, result: T::Result) -> Result<(), RclrsError> {
        let goal_info = GoalInfoHandle::new(self.handle);
        goal_info.lock().goal_id.uuid = goal_uuid;
        let server_handle = &*self.handle.lock();
        let goal_info_handle = &goal_info.lock();
        let goal_exists = unsafe {
            rcl_action_server_goal_exists(server_handle, goal_info_handle).ok()?
        };
        if (!goal_exists) {
            panic!("Asked to publish result for goal that does not exist");
        }

        { self.goal_results.lock().unwrap() }.insert(goal_uuid, result);

        match { self.result_requests.lock().unwrap() }.get(goal_uuid) {
            Some(req_ids) => {
                for req_id in req_ids {
                    unsafe {
                        rcl_action_send_result_response(server_handle, req_id, result).ok()?;
                    }
                }
            }
            None => {},
        }
    }

    pub fn notify_goal_terminal_state(&self) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        unsafe { rcl_action_notify_goal_done(handle).ok()?; }
    }

    pub fn call_handle_cancel_callback(&self, goal_uuid: GoalUUID) -> CancelResponse {
        let goal_handle_option = { self.goal_handles.lock().unwrap() }.get(goal_uuid);
        match goal_handle {
            Some(handle) => {
                let cancel_cb_response = (self.handle_cancel_cb)(handle);
                match cancel_cb_response {
                    CancelResponse::Accept => {
                        let result = handle.cancel_goal();
                        if result.is_err() {
                            return CancelResponse::Reject;
                        }
                    },
                    CancelResponse::Reject => CancelResponse::Reject
                }
            },
            None => CancelResponse::Reject
        }
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

pub struct CancelRequestHandle {
    rcl_action_cancel_request_mtx: Mutex<rcl_action_cancel_request_t>,
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>
}

impl CancelRequestHandle {
    pub fn new(goal_info: rcl_action_goal_info_t) -> Self {
        let rcl_action_cancel_req = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        Self {
            rcl_action_cancel_request_mtx: Mutex::new(rcl_action_cancel_req),
            rcl_action_goal_info_mtx: Mutex::new(goal_info)
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_response_t> {
        self.rcl_action_cancel_request_mtx.lock().unwrap();
    }
}
impl Drop for CancelRequestHandle {
    fn drop(&mut self) {
        let cancel_request = &mut *self.lock().unwrap();
        unsafe {
            rcl_action_cancel_request_fini(cancel_request);
        }
    }
}

pub struct CancelResponseHandle {
    rcl_action_cancel_response_mtx: Mutex<rcl_action_cancel_response_t>
}
impl CancelResponseHandle {
    pub fn new() -> Self {
        let rcl_action_cancel_response = unsafe { rcl_action_get_zero_initialized_cancel_response() };
        Self {
            rcl_action_cancel_response_mtx: Mutex::new(rcl_action_cancel_response),
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_response_t> {
        self.rcl_action_cancel_response_mtx.lock().unwrap();
    }
}

impl Drop for CancelResponseHandle {
    fn drop(&mut self) {
        let cancel_response = &mut *self.lock().unwrap();
        unsafe {
            rcl_action_cancel_response_fini(cancel_response);
        }
    }
}