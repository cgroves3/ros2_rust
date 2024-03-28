use rosidl_runtime_rs::Message;

use std::borrow::BorrowMut;
use std::collections::HashMap;
use std::ffi::CString;
use std::sync::{atomic::AtomicBool, atomic::Ordering, Arc, Mutex, MutexGuard};

use crate::error::{RclActionReturnCode, RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::vendor::action_msgs::msg::{GoalInfo, GoalStatus, GoalStatusArray};
use crate::vendor::action_msgs::srv::{CancelGoal_Request, CancelGoal_Response};
use crate::{rcl_bindings::*, MessageCow};

use crate::Clock;

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_server_t {}

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

pub struct ServerGoalHandleHandle
{
    rcl_goal_handle_mtx: Mutex<rcl_action_goal_handle_t>
}

impl ServerGoalHandleHandle {
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_handle_t> {
        self.rcl_goal_handle_mtx.lock().unwrap()
    }
}

impl Drop for ServerGoalHandleHandle
{
    fn drop(&mut self) {
        let goal_handle = &mut *self.rcl_goal_handle_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_goal_handle_fini(goal_handle);
        }
    }
}

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    handle: Arc<ServerGoalHandleHandle>,
    goal_request: Arc<T>,
    on_terminal_state: Box<dyn Fn(GoalUUID, T::Result) -> Result<(), RclrsError>>,
    on_executing: Box<dyn Fn(GoalUUID) -> Result<(), RclrsError>>,
    publish_feedback_cb: Box<dyn Fn(T::Feedback) -> Result<(), RclrsError>>
}

unsafe impl<T> Send for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

unsafe impl<T> Sync for ServerGoalHandle<T> where T: rosidl_runtime_rs::Action {}

impl<T> ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub fn new(
        handle: Arc<ServerGoalHandleHandle>,
        goal_request: Arc<T>,
        on_terminal_state: Box<dyn Fn(GoalUUID, T::Result) -> Result<(), RclrsError>>,
        on_executing: Box<dyn Fn(GoalUUID) -> Result<(), RclrsError>>,
        publish_feedback_cb: Box<dyn Fn(T::Feedback) -> Result<(), RclrsError>>,
    ) -> Self {
        Self {
            handle,
            goal_request: Arc::clone(&goal_request),
            on_terminal_state,
            on_executing,
            publish_feedback_cb
        }
    }

    pub fn publish_feedback(&self, feedback: T::Feedback) -> Result<(), RclrsError> {
        (self.publish_feedback_cb)(feedback)
    }

    fn get_state(&self) -> Result<i8, RclrsError> {
        let goal_handle = &*self.handle.lock();
        let mut state = &mut GoalStatus::STATUS_UNKNOWN;
        unsafe { rcl_action_goal_handle_get_status(goal_handle, &mut *state) }.ok()?;
        Ok(*state)
    }

    pub fn get_goal(&self) -> Arc<T> {
        self.goal_request
    }

    pub fn is_canceling(&self) -> bool {
        match self.get_state() {
            Ok(state) => GoalStatus::STATUS_CANCELING == state,
            Err(..) => false
        }
    }

    pub fn is_active(&self) -> bool {
        let handle = &*self.handle.lock();
        unsafe { rcl_action_goal_handle_is_active(handle) }
    }

    pub fn is_executing(&self) -> bool {
        match self.get_state() {
            Ok(state) => GoalStatus::STATUS_EXECUTING == state,
            Err(..) => false
        }
    }

    //TODO: Is `result` needed for these methods?
    pub fn abort(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = &mut *self.handle.lock();
        unsafe { 
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_ABORT) 
        }
        .ok()?;
        Ok(())
    }

    pub fn succeed(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = &mut *self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_SUCCEED)
        }
        .ok()?;
        Ok(())
    }

    pub fn cancel_goal(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = &mut *self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_CANCEL_GOAL)
        }
        .ok()?;
        Ok(())
    }

    pub fn canceled(&self, result: &T::Result) -> Result<(), RclrsError> {
        let handle = &mut *self.handle.lock();
        unsafe {
            rcl_action_update_goal_state(handle, rcl_action_goal_event_e::GOAL_EVENT_CANCELED)
        }
        .ok()?;
        Ok(())
    }
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
    goal_expired: Arc<AtomicBool>
}

impl<T> ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action server.
    pub(crate) fn new(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        name: &str,
        clock: Clock,
        qos: QoSProfile,
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
            <T as rosidl_runtime_rs::Action>::get_type_support() as *const rosidl_action_type_support_t;
        let name_c_string = CString::new(name).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: name.into(),
        })?;

        // SAFETY: No preconditions for this function.
        let mut action_server_options = unsafe { rcl_action_server_get_default_options() };
        action_server_options.goal_service_qos = qos.into();
        action_server_options.cancel_service_qos = qos.into();
        action_server_options.result_service_qos = qos.into();
        action_server_options.feedback_topic_qos = qos.into();
        action_server_options.status_topic_qos = qos.into();
        let server_clock = &mut clock.get_clock_handle();
        unsafe {
            // SAFETY: The rcl_action_server is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
            // afterwards.
            // TODO: type support?
            rcl_action_server_init(
                &mut rcl_action_server,
                &mut *rcl_node_mtx.lock().unwrap(),
                server_clock,
                type_support_ptr,
                name_c_string.as_ptr(),
                &action_server_options,
            )
            .ok()?;
        }

        let handle = Arc::new(ActionServerHandle {
            rcl_action_server_mtx: Mutex::new(rcl_action_server),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false))
        });

        Ok(Self {
            goal_handles: Arc::new(Mutex::new(HashMap::new())),
            goal_results: Arc::new(Mutex::new(HashMap::new())),
            result_requests: Arc::new(Mutex::new(HashMap::new())),
            handle,
            handle_goal_cb,
            handle_cancel_cb,
            handle_accepted_cb,
            goal_request_ready: Arc::new(AtomicBool::new(false)),
            cancel_request_ready: Arc::new(AtomicBool::new(false)),
            result_request_ready: Arc::new(AtomicBool::new(false)),
            goal_expired: Arc::new(AtomicBool::new(false))
        })
    }

    pub fn publish_feedback(&self, message: T::Feedback) -> Result<(), RclrsError> {
        let rmw_message = <T::Feedback as Message>::into_rmw_message(message.into_cow());
        let rcl_action_server = &mut *self.handle.lock();
        unsafe {
            // SAFETY: The message type is guaranteed to match the publisher type by the type system.
            // The message does not need to be valid beyond the duration of this function call.
            // The third argument is explictly allowed to be NULL.
            rcl_action_publish_feedback(
                rcl_action_server,
                rmw_message.as_ref() as *const <T::Feedback as Message>::RmwMsg as *mut _,
            )
            .ok()?;
        }
        Ok(())
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
            )
            .ok()?;
        }
        Ok((T::Goal::from_rmw_message(request_out), request_id_out))
    }


    pub fn take_result_request(&self) -> Result<(T::Result, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<T as rosidl_runtime_rs::Action>::Result as rosidl_runtime_rs::Message>::RmwMsg;
        let mut request_out = RmwMsg::<T>::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_result_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg<T> as *mut _,
            )
            .ok()?
        }
        Ok((T::Result::from_rmw_message(request_out), request_id_out))
    }

    pub fn take_cancel_request(
        &self,
    ) -> Result<(CancelGoal_Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        let mut request_out = <CancelGoal_Request as Message>::RmwMsg::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_cancel_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut <CancelGoal_Request as Message>::RmwMsg as *mut _,
            )
            .ok()?
        }
        Ok((CancelGoal_Request::from_rmw_message(request_out), request_id_out))
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

        let res = (self.handle_goal_cb)(&goal_request.uuid, goal_request.into());

        if matches!(res, GoalResponse::AcceptAndExecute | GoalResponse::AcceptAndDefer) {
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
            }
            .ok()?;
            // TODO: rclcpp also inserts into a map of GoalUUID and rcl_action_goal_handle_t pairs for some reason. Unsure if this is needed
            if matches!(res, GoalResponse::AcceptAndExecute) {
                unsafe {
                    rcl_action_update_goal_state(new_goal_handle, rcl_action_goal_event_e::Execute);
                }
                .ok()?;
            }
            self.publish_status()?;
            self.call_goal_accepted_cb(new_goal_handle, uuid, goal_request);
        }
        Ok(())
    }

    pub fn call_goal_accepted_cb(
        &self,
        goal_handle: rcl_action_goal_handle_t,
        goal_uuid: GoalUUID,
        goal_request: T::Goal,
    ) -> () {
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

        let publish_feedback = |feedback_msg: T::Feedback| -> Result<(), RclrsError> {
            self.publish_feedback(feedback_msg);
        };

        let goal_handle_handle = ServerGoalHandleHandle {
            rcl_goal_handle_mtx: Mutex::new(goal_handle)
        };

        let goal_handle_arc = Arc::new(ServerGoalHandle::new(
            Arc::new(goal_handle_handle),
            Arc::new(goal_request),
            Box::new(on_terminal_state),
            Box::new(on_executing),
            Box::new(publish_feedback),
        ));
        { self.goal_handles.lock().unwrap() }.insert(goal_uuid, goal_handle_arc);
        (self.handle_accepted_cb)(goal_handle_arc);
    }

    pub fn execute_cancel_request_received(&self) -> Result<(), RclrsError> {
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
        let mut cancel_response_handle = CancelResponseHandle::new();
        let handle = &*self.handle.lock();
        let request_handle = cancel_request_handle.lock().unwrap();
        let response_handle = cancel_response_handle.lock().unwrap();
        unsafe {
            rcl_action_process_cancel_request(handle, &request_handle, &mut cancel_response_handle)
        }
        .ok()?;
        let response = Arc::new(CancelGoal_Response::default());
        response.return_code = response_handle.msg.return_code;
        let goals = response_handle.msg.goals_canceling;

        for i in 0..goals.len() {
            let goal_info = goals.data[i];
            let uuid = goal_info.uuid;
            let cb_response_code = self.call_handle_cancel_callback(uuid);
            if CancelResponse::Accept == cb_response_code {
                let mut rs_goal_info = GoalInfo::default();
                rs_goal_info.goal_id.uuid = goal_info.uuid;
                //TODO: Set the timestamp for the rs_goal_info
                response.goals_canceling.push(rs_goal_info);
            }
        }

        // If the user rejects all individual requests to cancel goals,
        // then we consider the top-level cancel request as rejected.
        if goals.len() >= 1 && 0 == response.goals_canceling.len() {
            response.return_code = CancelResponse::Reject;
        }

        if !response.goals_canceling.empty() {
            // at least one goal state changed, publish a new status message
            self.publish_status();
        }

        let rmw_message = <T::Response as Message>::into_rmw_message(response.into_cow());
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_action_send_cancel_response(
                handle,
                &mut req_id,
                rmw_message.as_ref() as *const <T::Response as Message>::RmwMsg as *mut _,
            )
        }
        .ok()?;

        Ok(())
    }
    pub fn execute_result_request_received(&self) -> Result<(), RclrsError> {
        let (result_request, mut req_id) = match self.take_result_request() {
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

        self.result_request_ready.store(false, Ordering::SeqCst);
        // TODO: Convert the result_request uuid to a uuid
        let goal_uuid = result_request.uuid;
        let mut goal_info_handle = &*GoalInfoHandle::new(self.handle);

        let handle = self.handle.lock().unwrap();
        let mut goal_exists = unsafe { rcl_action_server_goal_exists(handle, goal_info_handle) };
        if !goal_exists {
            type ResultResponse<T> = <<<T as rosidl_runtime_rs::Action>::GetResult as rosidl_runtime_rs::Service>::Response as rosidl_runtime_rs::Message>::RmwMsg;
            //TODO: set this to unknown
            let mut result_response = ResultResponse::<T>::default();
            unsafe {
                rcl_action_send_result_response(handle, &mut req_id, result_response);
            }
            .ok()?;
        } else {
            if let Some(result_response) = { self.goal_results.lock().unwrap() }.get_mut(goal_uuid)
            {
                unsafe {
                    rcl_action_send_result_response(handle, &mut req_id, result_response)
                }
                .ok()?;
            } else {
                if let Some(request_headers) = { self.result_requests.lock().unwrap() }.get_mut(goal_uuid)
                {
                    request_headers.push_back(req_id);
                }
            }
        }
        Okay(())
    }

    pub fn execute_check_expired_goals(&self) -> Result<(), RclrsError> {
        let mut goal_info_handle = &*GoalInfoHandle::new(self.handle);
        let mut num_expired: &*mut u32 = &1;
        while num_expired > 0 {
            let handle = self.handle.lock().unwrap();
            unsafe { rcl_action_expire_goals(handle, goal_info_handle, 1, num_expired) }.ok()?;
            if num_expired > 0 {
                // TODO: Convert the expired goal_info uuid to a uuid
                let goal_uuid = goal_info_handle.uuid;
                { self.goal_results.lock().unwrap() }.remove(goal_uuid);
                { self.result_requests.lock().unwrap() }.remove(goal_uuid);
                { self.goal_handles.lock().unwrap() }.remove(goal_uuid);
            }
        }
    }

    pub fn publish_status(&self) -> Result<(), RclrsError> {
        let mut num_goals: usize = 0;
        let mut goal_handles = std::ptr::null_mut();
        let handle = &*self.handle.lock();
        unsafe { rcl_action_server_get_goal_handles(handle, goal_handles, num_goals) }.ok()?;
        let goal_status_array = Arc::new(GoalStatusArray::default());
        goal_status_array.reserve(num_goals);
        let mut status_array = unsafe { rcl_action_get_zero_initialized_goal_status_array() };
        unsafe { rcl_action_get_goal_status_array(handle, &mut status_array) }.ok()?;
        unsafe { rcl_action_goal_status_array_fini(&mut status_array) }.ok()?;
        for i in 0..status_array.msg.status_list.len() {
            let status_msg = status_array.msg.status_list[i];

            let goal_status = GoalStatus::default();
            goal_status.status = status_msg.status;
            goal_status.goal_info.stamp = status_msg.stamp;
            goal_status.uuid = status_msg.goal_info.uuid;
            goal_status_array.status_list.append(goal_status);
        }
        unsafe { rcl_action_publish_status(handle, goal_status_array) }.ok()?;
    }

    pub fn accept_new_goal(
        &self,
        goal_info: GoalInfoHandle,
        goal_req: Arc<T>,
    ) -> Result<rcl_action_goal_handle_t, RclrsError> {
        let handle = &*self.handle.lock();
        let goal_info_handle = &goal_info.lock();
        let goal_handle = unsafe { rcl_action_accept_new_goal(handle, goal_info_handle).ok()? };
    }

    pub fn publish_result(&self, goal_uuid: GoalUUID, result: T::Result) -> Result<(), RclrsError> {
        let goal_info = GoalInfoHandle::new(self.handle);
        goal_info.lock().goal_id.uuid = goal_uuid;
        let server_handle = &*self.handle.lock();
        let goal_info_handle = &goal_info.lock();
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(server_handle, goal_info_handle).ok()? };
        if !goal_exists {
            panic!("Asked to publish result for goal that does not exist");
        }

        { self.goal_results.lock().unwrap() }.insert(goal_uuid, result);

        if let Some(req_ids) = { self.result_requests.lock().unwrap() }.get(goal_uuid) {
            for req_id in req_ids {
                unsafe {
                    rcl_action_send_result_response(server_handle, req_id, result).ok()?;
                }
            }
        }
    }

    pub fn notify_goal_terminal_state(&self) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_notify_goal_done(handle).ok()?;
        }
    }

    pub fn call_handle_cancel_callback(&self, goal_uuid: GoalUUID) -> CancelResponse {
        let goal_handle_option = { self.goal_handles.lock().unwrap() }.get(goal_uuid);
        match goal_handle_option {
            Some(handle) => {
                let cancel_cb_response = (self.handle_cancel_cb.lock().unwrap())(handle);
                match cancel_cb_response {
                    CancelResponse::Accept => {
                        let result = handle.cancel_goal();
                        if result.is_err() {
                            return CancelResponse::Reject;
                        }
                    }
                    CancelResponse::Reject => CancelResponse::Reject,
                }
            }
            None => CancelResponse::Reject,
        }
    }
}

pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>,
    rcl_action_server: Arc<ActionServerHandle>,
}

impl GoalInfoHandle {
    pub fn new(rcl_action_server: Arc<ActionServerHandle>) -> Self {
        let mut goal_info = unsafe { rcl_action_get_zero_initialized_goal_info };
        Self {
            rcl_action_goal_info_mtx: Mutex::new(goal_info),
            rcl_action_server,
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_goal_info_t> {
        self.rcl_action_goal_info_mtx.lock().unwrap()
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

impl<T> ActionServerBase for ActionServer<T> 
where T: rosidl_runtime_rs::Action {
    fn handle(&self) -> &ActionServerHandle {
        &self.handle
    }
    fn execute(&self) -> Result<(), RclrsError> {
        if self.goal_request_ready.load(Ordering::SeqCst) {
            self.execute_goal_request_received();
        } else if self.cancel_request_ready.load(Ordering::SeqCst) {
            self.execute_cancel_request_received();
        } else if self.result_request_ready.load(Ordering::SeqCst) {
            self.execute_result_request_received();
        } else if self.goal_expired.load(Ordering::SeqCst) {
            self.execute_check_expired_goals();
        } else {
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
        self.goal_request_ready.load(Ordering::SeqCst)
            || self.cancel_request_ready.load(Ordering::SeqCst)
            || self.result_request_ready.load(Ordering::SeqCst)
            || self.goal_expired.load(Ordering::SeqCst);
    }
}

pub struct CancelRequestHandle {
    rcl_action_cancel_request_mtx: Mutex<rcl_action_cancel_request_t>,
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>,
}

impl CancelRequestHandle {
    pub fn new(goal_info: rcl_action_goal_info_t) -> Self {
        let rcl_action_cancel_req = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        Self {
            rcl_action_cancel_request_mtx: Mutex::new(rcl_action_cancel_req),
            rcl_action_goal_info_mtx: Mutex::new(goal_info),
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_response_t> {
        self.rcl_action_cancel_request_mtx.lock().unwrap();
    }
}

pub struct CancelResponseHandle {
    rcl_action_cancel_response_mtx: Mutex<rcl_action_cancel_response_t>,
}
impl CancelResponseHandle {
    pub fn new() -> Self {
        let rcl_action_cancel_response =
            unsafe { rcl_action_get_zero_initialized_cancel_response() };
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
