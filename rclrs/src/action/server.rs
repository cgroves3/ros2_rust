use rosidl_runtime_rs::{
    Accepted, Action, GetResultService, HasGoal, HasGoalId, Message, SendGoalService, SetResult, Status
};

use std::collections::HashMap;
use std::ffi::CString;
use std::sync::{atomic::AtomicBool, atomic::Ordering, Arc, Mutex, MutexGuard};

use crate::action::GoalUUID;
use crate::action::types::*;
use crate::error::{RclActionReturnCode, RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::server_goal_handle::{ServerGoalHandle, ServerGoalHandleHandle};
use crate::Clock;
use crate::{rcl_bindings::*, MessageCow};

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_server_t {}

// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_goal_handle_t {}

unsafe impl Sync for rcl_action_goal_handle_t {}

/// The goal response states
pub enum GoalResponse {
    /// The reject goal response state
    Reject = 1,
    /// The accept and execute goal response state
    AcceptAndExecute = 2,
    /// The accept and defer goal response state
    AcceptAndDefer = 3,
}

/// The cancel response states
pub enum CancelResponse {
    /// The reject cancel response state
    Reject = 1,
    /// The accept cancel response state
    Accept = 2,
}

/// Internal struct used by action servers.
#[derive(Debug)]
pub struct ActionServerHandle {
    rcl_action_server_mtx: Mutex<rcl_action_server_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
    pub(crate) in_use_by_wait_set: Arc<AtomicBool>
}

impl ActionServerHandle {
    /// Locks and unwraps the action server
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_server_t> {
        self.rcl_action_server_mtx.lock().unwrap()
    }
    
    /// Publishes the feedback message
    pub fn publish_feedback<T>(&self, message: T::Feedback) -> Result<(), RclrsError> 
    where T: rosidl_runtime_rs::Action {
        let rmw_message = <T::Feedback as Message>::into_rmw_message(message.into_cow());
        let rcl_action_server = &mut *self.lock();
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

/// The struct representing the ROS Action Server
pub struct ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    pub(crate) goal_handles_mtx:
        Arc<Mutex<HashMap<GoalUUID, Arc<Mutex<ServerGoalHandle<T>>>>>>,
    pub(crate) goal_results: Arc<
        Mutex<HashMap<GoalUUID, <T::GetResult as GetResultService>::Response>>,
    >,
    pub(crate) result_requests: Arc<Mutex<HashMap<GoalUUID, Vec<rmw_request_id_t>>>>,
    pub(crate) handle: Arc<ActionServerHandle>,
    handle_goal_cb: fn(&GoalUUID, Arc<<T as Action>::Goal>) -> GoalResponse,
    handle_cancel_cb: fn(Arc<Mutex<ServerGoalHandle<T>>>) -> CancelResponse,
    handle_accepted_cb: fn(Arc<Mutex<ServerGoalHandle<T>>>),
    goal_request_ready: Arc<AtomicBool>,
    cancel_request_ready: Arc<AtomicBool>,
    result_request_ready: Arc<AtomicBool>,
    goal_expired: Arc<AtomicBool>,
    num_subscriptions: usize,
    num_guard_conditions: usize,
    num_timers: usize,
    num_clients: usize,
    num_services: usize
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
        handle_goal_cb: fn(&GoalUUID, Arc<T::Goal>) -> GoalResponse,
        handle_cancel_cb: fn(Arc<Mutex<ServerGoalHandle<T>>>) -> CancelResponse,
        handle_accepted_cb: fn(Arc<Mutex<ServerGoalHandle<T>>>),
    ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_server = unsafe { rcl_action_get_zero_initialized_server() };
        let type_support_ptr = <T as rosidl_runtime_rs::Action>::get_type_support()
            as *const rosidl_action_type_support_t;
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
        let server_clock = &mut *clock.lock();
        unsafe {
            // SAFETY: The rcl_action_server is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the action server.
            // The action name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_action_server_init(
                &mut rcl_action_server,
                &mut *rcl_node_mtx.lock().unwrap(),
                server_clock as *mut _,
                type_support_ptr,
                name_c_string.as_ptr(),
                &action_server_options,
            )
            .ok()?;
        }

        let mut num_subscriptions = 0;
        let mut num_guard_conditions = 0;
        let mut num_timers = 0;
        let mut num_clients = 0;
        let mut num_services = 0;

        unsafe {
            rcl_action_client_wait_set_get_num_entities(
                &rcl_action_server,
                &mut num_subscriptions,
                &mut num_guard_conditions,
                &mut num_timers,
                &mut num_clients,
                &mut num_services
            ).ok()?;
        }

        let handle = Arc::new(ActionServerHandle {
            rcl_action_server_mtx: Mutex::new(rcl_action_server),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false)),
        });

        Ok(Self {
            goal_handles_mtx: Arc::new(Mutex::new(HashMap::new())),
            goal_results: Arc::new(Mutex::new(HashMap::new())),
            result_requests: Arc::new(Mutex::new(HashMap::new())),
            handle,
            handle_goal_cb,
            handle_cancel_cb,
            handle_accepted_cb,
            goal_request_ready: Arc::new(AtomicBool::new(false)),
            cancel_request_ready: Arc::new(AtomicBool::new(false)),
            result_request_ready: Arc::new(AtomicBool::new(false)),
            goal_expired: Arc::new(AtomicBool::new(false)),
            num_subscriptions: num_subscriptions,
            num_guard_conditions: num_guard_conditions,
            num_timers: num_timers,
            num_clients: num_clients,
            num_services: num_services,
        })
    }

    /// Takes a goal request
    pub fn take_goal_request(
        &self,
    ) -> Result<
        (
            <<T as Action>::SendGoal as SendGoalService>::Request,
            rmw_request_id_t,
        ),
        RclrsError,
    > {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> = <<<T as Action>::SendGoal as SendGoalService>::Request as Message>::RmwMsg;
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
        Ok((
            <<<T as Action>::SendGoal as SendGoalService>::Request>::from_rmw_message(request_out),
            request_id_out,
        ))
    }

    /// Takes a result request
    pub fn take_result_request(
        &self,
    ) -> Result<
        (
            <T::GetResult as GetResultService>::Request,
            rmw_request_id_t,
        ),
        RclrsError,
    > {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        // type RmwMsg<T> =
        //     <<T as rosidl_runtime_rs::Action>::Result as rosidl_runtime_rs::Message>::RmwMsg;
        type RmwMsg<T> = <<<T as Action>::GetResult as GetResultService>::Request as rosidl_runtime_rs::Message>::RmwMsg;
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
        Ok((
            <T::GetResult as GetResultService>::Request::from_rmw_message(request_out),
            request_id_out,
        ))
    }

    /// Takes a cancel request
    pub fn take_cancel_request(
        &self,
    ) -> Result<(crate::vendor::action_msgs::srv::CancelGoal_Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };

        type RmwMsg =
            <crate::vendor::action_msgs::srv::CancelGoal_Request as rosidl_runtime_rs::Message>::RmwMsg;
        
        let mut request_out = RmwMsg::default();
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_take_cancel_request(
                handle,
                &mut request_id_out,
                &mut request_out as *mut RmwMsg as *mut _,
            )
            .ok()?
        }
        Ok((
            crate::vendor::action_msgs::srv::CancelGoal_Request::from_rmw_message(request_out),
            request_id_out,
        ))
    }

    /// Takes a goal request and processes it
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

        let goal = Arc::new(goal_request.get_goal::<T>());
        let uuid = GoalUUID::new(goal_request.get_goal_id());
        let user_response = (self.handle_goal_cb)(&uuid, goal);

        type Response<T> = <<T as Action>::SendGoal as SendGoalService>::Response;
        let mut goal_response = Response::<T>::default();
        let goal_accepted = matches!(
            user_response,
            GoalResponse::AcceptAndExecute | GoalResponse::AcceptAndDefer
        );
        goal_response.set_accepted(goal_accepted);
        let goal_response_rmw = <Response::<T> as Message>::into_rmw_message(goal_response.into_cow());
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_action_send_goal_response(
                handle,
                &mut req_id,
                goal_response_rmw.as_ref() as *const <Response::<T> as Message>::RmwMsg as *mut _,
            )
        }
        .ok()?;

        if goal_accepted {
            let goal_info_handle = GoalInfoHandle::new();
            let uuid = GoalUUID::new(goal_request.get_goal_id());
            goal_info_handle
                .lock()
                .goal_id
                .uuid
                .copy_from_slice(&uuid.0);

            let goal_handle_handle = self.accept_new_goal(goal_info_handle);

            if matches!(user_response, GoalResponse::AcceptAndExecute) {
                let goal_handle = goal_handle_handle.lock();
                unsafe {
                    rcl_action_update_goal_state(
                        *goal_handle,
                        rcl_action_goal_event_e::GOAL_EVENT_EXECUTE,
                    )
                }
                .ok()?;
            }
            self.publish_status()?;

            let server_goal_handle_mtx = Arc::new(
                Mutex::new(
                    ServerGoalHandle::<T>::new(
                        Arc::new(goal_handle_handle),
                        <T as Action>::Result::default(),
                        Arc::new(goal_request.get_goal::<T>()),
                        self.handle.clone()
                    )
                )
            );
            (self.handle_accepted_cb)(server_goal_handle_mtx.clone());

            let goal_status = { server_goal_handle_mtx.lock().unwrap() }.get_status()?;
            if ServerGoalHandle::<T>::TERMINAL_STATES.contains(&goal_status) {
                let server_goal_handle_unwrapped = Arc::try_unwrap(server_goal_handle_mtx);
                match server_goal_handle_unwrapped {
                    Ok(handle_mtx) => {
                        let server_goal_handle = handle_mtx.into_inner().unwrap();
                        type Response<T> = <<T as Action>::GetResult as GetResultService>::Response;
                        let mut result_response = Response::<T>::default();
                        result_response.set_status(goal_status);
                        // TODO: Refactor so the server_goal_handle doesn't need to be owned.
                        result_response.set_result::<T>(server_goal_handle.result);
                        self.publish_result(&uuid, result_response)?;
                        self.publish_status()?;
                        self.notify_goal_terminal_state()?;
                    }
                    Err(server_goal_handle_failed_mtx) => {
                    }
                }
            } else {
                { self.goal_handles_mtx.lock().unwrap() }.insert(uuid.clone(), server_goal_handle_mtx);
            }
            if goal_status == GoalStatus::STATUS_EXECUTING {
                self.publish_status()?;
            }
        }
        Ok(())
    }

    /// Accepts a new server goal handle 
    pub fn accept_new_goal(&self, goal_info_handle: GoalInfoHandle) -> ServerGoalHandleHandle {
        let mut_handle = &mut *self.handle.lock();
        let goal_info = &*goal_info_handle.rcl_action_goal_info_mtx.lock().unwrap();
        unsafe {
            let goal_handle_raw_ptr = rcl_action_accept_new_goal(mut_handle, goal_info);
            ServerGoalHandleHandle::new(Mutex::new(goal_handle_raw_ptr))
        }
    }

    /// Takes a cancel request and processes it
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
        let cancel_request_handle = CancelRequestHandle::new();
        {cancel_request_handle.lock()}.goal_info.goal_id.uuid.copy_from_slice(&cancel_request.goal_info.goal_id.uuid);
        {cancel_request_handle.lock()}.goal_info.stamp.sec = cancel_request.goal_info.stamp.sec;
        {cancel_request_handle.lock()}.goal_info.stamp.nanosec = cancel_request.goal_info.stamp.nanosec;
        let cancel_response_handle = CancelResponseHandle::new();
        let handle = &*self.handle.lock();
        let request_handle = &*cancel_request_handle.lock();
        let response_handle = &mut *cancel_response_handle.lock();
        unsafe {
            rcl_action_process_cancel_request(handle, request_handle, response_handle as *mut _)
        }
        .ok()?;
        // The response code should have been written to, so there is no need to copy it into the CancelResponseHandle
    
        let goals = unsafe {
            std::slice::from_raw_parts(
                response_handle.msg.goals_canceling.data,
                response_handle.msg.goals_canceling.size,
            )
        };

        let mut response_rs = crate::vendor::action_msgs::srv::CancelGoal_Response::default();
        for i in 0..goals.len() {
            let goal_info = &goals[i];
            let mut uuid = GoalUUID::new([0; RCL_ACTION_UUID_SIZE]);
            uuid.0.copy_from_slice(&goal_info.goal_id.uuid);
            let cb_response_code = self.call_handle_cancel_callback(uuid);
            if matches!(cb_response_code, CancelResponse::Accept) {
                let mut goal_info_rs = crate::vendor::action_msgs::msg::GoalInfo::default();
                goal_info_rs
                    .goal_id
                    .uuid
                    .clone_from_slice(&goal_info.goal_id.uuid);
                goal_info_rs.stamp.sec = goal_info.stamp.sec;
                goal_info_rs.stamp.nanosec = goal_info.stamp.nanosec;
                response_rs.goals_canceling.push(goal_info_rs);
            }
        }

        // If the user rejects all individual requests to cancel goals,
        // then we consider the top-level cancel request as rejected.
        if goals.len() >= 1 && 0 == response_rs.goals_canceling.len() {
            response_rs.return_code = CancelResponse::Reject as i8;
        }

        // Probably overkill
        if !response_rs.goals_canceling.is_empty() {
            // at least one goal state changed, publish a new status message
            self.publish_status()?;
        }

        {
            type CancelGoalResponse = crate::vendor::action_msgs::srv::CancelGoal_Response;
            let rmw_response = CancelGoalResponse::into_rmw_message(response_rs.into_cow());
            unsafe {
                // SAFETY: The response type is guaranteed to match the service type by the type system.
                rcl_action_send_cancel_response(
                    handle,
                    &mut req_id,
                    rmw_response.as_ref() as *const <CancelGoalResponse as Message>::RmwMsg as *mut _,
                )
            }
        }
        .ok()?;

        Ok(())
    }

    /// Takes a result request and processes it.
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
        let goal_uuid = GoalUUID::new(result_request.get_goal_id());
        let goal_info_handle = GoalInfoHandle::new();

        let handle = &*self.handle.lock();
        let goal_exists = unsafe { rcl_action_server_goal_exists(handle, &*goal_info_handle.lock()) };
        type RmwMsg<T> = <<<T as Action>::GetResult as GetResultService>::Response as Message>::RmwMsg;
        if !goal_exists {
            type Response<T> = <<T as Action>::GetResult as GetResultService>::Response;
            let mut result_response = Response::<T>::default();
            result_response.set_status(GoalStatus::STATUS_UNKNOWN);
            let result_rmw_message = <<<T as Action>::GetResult as GetResultService>::Response as Message>::into_rmw_message(result_response.into_cow());
            unsafe {
                rcl_action_send_result_response(
                    handle,
                    &mut req_id,
                    result_rmw_message.as_ref() as *const RmwMsg<T> as *mut _,
                )
            }
            .ok()?;
        } else {
            // Goal exists, check if a result is already available
            if let Some(result_response) = { self.goal_results.lock().unwrap() }.remove(&goal_uuid)
            {
                let rmw_message = <<<T as Action>::GetResult as GetResultService>::Response as Message>::into_rmw_message(result_response.into_cow());
                unsafe {
                    rcl_action_send_result_response(
                        handle,
                        &mut req_id,
                        rmw_message.as_ref() as *const RmwMsg<T> as *mut _,
                    )
                }
                .ok()?;
            } else {
                if let Some(request_headers) =
                    { self.result_requests.lock().unwrap() }.get_mut(&goal_uuid)
                {
                    request_headers.push(req_id);
                }
            }
        }
        Ok(())
    }

    /// Checks for expired goals and removes them from the goal results, result request and goal handles
    pub fn execute_check_expired_goals(&self) -> Result<(), RclrsError> {
        let goal_info_handle = GoalInfoHandle::new();
        let mut num_expired: usize = 1;
        while num_expired > 0 {
            let handle = &*self.handle.lock();
            unsafe {
                rcl_action_expire_goals(handle, &mut *goal_info_handle.lock(), 1, &mut num_expired)
            }
            .ok()?;
            if num_expired > 0 {
                let mut goal_uuid = GoalUUID::new([0; RCL_ACTION_UUID_SIZE]);
                goal_uuid.0.copy_from_slice(&goal_info_handle.lock().goal_id.uuid);
                { self.goal_results.lock().unwrap() }.remove(&goal_uuid);
                { self.result_requests.lock().unwrap() }.remove(&goal_uuid);
                { self.goal_handles_mtx.lock().unwrap() }.remove(&goal_uuid);
            }
        }
        Ok(())
    }

    /// Publish the status of all goal handles
    pub fn publish_status(&self) -> Result<(), RclrsError> {
        let mut num_goals: usize = 0;
        let goal_handles = std::ptr::null_mut();
        let handle = &*self.handle.lock();
        // Here goal_handles: *mut *mut *mut rcl_action_goal_handle_t
        unsafe { rcl_action_server_get_goal_handles(handle, goal_handles, &mut num_goals) }.ok()?;
        let rcl_goal_status_array = unsafe { rcl_action_get_zero_initialized_goal_status_array() };
        let goal_status_array_c = GoalStatusArrayHandle::new(rcl_goal_status_array);
        let goal_status_array_handle = &mut *goal_status_array_c.lock();
        unsafe { rcl_action_get_goal_status_array(handle, goal_status_array_handle) }.ok()?;
        let status_array_slice = unsafe {
            std::slice::from_raw_parts(
                goal_status_array_handle.msg.status_list.data,
                goal_status_array_handle.msg.status_list.size,
            )
        };

        let mut goal_status_array_rs = crate::vendor::action_msgs::msg::GoalStatusArray::default();
        goal_status_array_rs.status_list.reserve(num_goals);
        for i in 0..status_array_slice.len() {
            let c_status_msg = &status_array_slice[i];

            let mut goal_status_rs = crate::vendor::action_msgs::msg::GoalStatus::default();
            goal_status_rs.status = c_status_msg.status;
            goal_status_rs.goal_info.stamp = crate::vendor::builtin_interfaces::msg::Time {
                sec: c_status_msg.goal_info.stamp.sec,
                nanosec: c_status_msg.goal_info.stamp.nanosec,
            };
            goal_status_rs
                .goal_info
                .goal_id
                .uuid
                .copy_from_slice(&c_status_msg.goal_info.goal_id.uuid);
            goal_status_array_rs.status_list.push(goal_status_rs);
        }
        let goal_status_array_rmw_msg = crate::vendor::action_msgs::msg::GoalStatusArray::into_rmw_message(
                goal_status_array_rs.into_cow()
        );
        unsafe {
            rcl_action_publish_status(
                handle,
                goal_status_array_rmw_msg.as_ref() as *const <crate::vendor::action_msgs::msg::GoalStatusArray as Message>::RmwMsg as *mut _
            )
        }
        .ok()?;
        Ok(())
    }

    /// Publishes the given result for the given goal uuid
    pub fn publish_result(
        &self,
        goal_uuid: &GoalUUID,
        result: <T::GetResult as GetResultService>::Response,
    ) -> Result<(), RclrsError> {
        let goal_info = GoalInfoHandle::new();
        goal_info.lock().goal_id.uuid.copy_from_slice(&goal_uuid.0);
        let server_handle = &*self.handle.lock();
        let goal_info_handle = &*goal_info.lock();
        let goal_exists = unsafe { rcl_action_server_goal_exists(server_handle, goal_info_handle) };
        if !goal_exists {
            panic!("Asked to publish result for goal that does not exist");
        }
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
    pub fn notify_goal_terminal_state(&self) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        unsafe { rcl_action_notify_goal_done(handle) }.ok()?;
        Ok(())
    }

    /// Calls the user-defined handle cancel callback
    pub fn call_handle_cancel_callback(&self, goal_uuid: GoalUUID) -> CancelResponse {
        let goal_handles = self.goal_handles_mtx.lock().unwrap();
        let goal_handle_option = goal_handles.get(&goal_uuid);
        match goal_handle_option {
            Some(goal_handle_arc) => {
                let cancel_cb_response = (self.handle_cancel_cb)(goal_handle_arc.clone());
                match cancel_cb_response {
                    CancelResponse::Accept => {
                        let result = { goal_handle_arc.lock().unwrap() }.cancel_goal();
                        if result.is_err() {
                            CancelResponse::Reject
                        } else {
                            CancelResponse::Accept
                        }
                    }
                    CancelResponse::Reject => CancelResponse::Reject,
                }
            }
            None => CancelResponse::Reject,
        }
    }
}

/// A handle for the goal info
pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>
}

impl GoalInfoHandle {
    /// Creates a new goal info
    pub fn new() -> Self {
        let goal_info = unsafe { rcl_action_get_zero_initialized_goal_info() };
        Self {
            rcl_action_goal_info_mtx: Mutex::new(goal_info)
        }
    }

    /// Locks and unwraps the goal info
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

    // /// Sets the goal request ready state to value
    // fn set_goal_request_ready(&self, value: bool) -> ();
    // /// Sets the cancel request ready state to value
    // fn set_cancel_request_ready(&self, value: bool) -> ();
    // /// Sets the result request ready state to value
    // fn set_result_request_ready(&self, value: bool) -> ();
    // /// Sets the goal expired state to value
    // fn set_goal_expired(&self, value: bool) -> ();

    /// Returns if the any requests are ready
    fn is_ready(&self, wait_set: &mut rcl_wait_set_t) -> bool;
}

impl<T> ActionServerBase for ActionServer<T>
where
    T: rosidl_runtime_rs::Action,
{
    fn handle(&self) -> &ActionServerHandle {
        &self.handle
    }

    fn execute(&self) -> Result<(), RclrsError> {
        if self.goal_request_ready.load(Ordering::SeqCst) {
            self.execute_goal_request_received()
        } else if self.cancel_request_ready.load(Ordering::SeqCst) {
            self.execute_cancel_request_received()
        } else if self.result_request_ready.load(Ordering::SeqCst) {
            self.execute_result_request_received()
        } else if self.goal_expired.load(Ordering::SeqCst) {
            self.execute_check_expired_goals()
        } else {
            panic!("Executing action server but nothing is ready")
        }
    }

    // fn set_goal_request_ready(&self, value: bool) -> () {
    //     self.goal_request_ready.store(value, Ordering::SeqCst)
    // }
    // fn set_cancel_request_ready(&self, value: bool) -> () {
    //     self.cancel_request_ready.store(value, Ordering::SeqCst)
    // }
    // fn set_result_request_ready(&self, value: bool) -> () {
    //     self.result_request_ready.store(value, Ordering::SeqCst)
    // }
    // fn set_goal_expired(&self, value: bool) -> () {
    //     self.goal_expired.store(value, Ordering::SeqCst)
    // }

    fn is_ready(&self, wait_set: &mut rcl_wait_set_t) -> bool {
        let mut goal_request_ready = false;
        let mut cancel_request_ready = false;
        let mut result_request_ready = false;
        let mut goal_expired = false;

        unsafe {
            rcl_action_server_wait_set_get_entities_ready(
                wait_set,
                &*self.handle().lock() as *const _,
                &mut goal_request_ready,
                &mut cancel_request_ready,
                &mut result_request_ready,
                &mut goal_expired,
            )
        };

        self.goal_request_ready
            .store(goal_request_ready, Ordering::SeqCst);
        self.cancel_request_ready
            .store(cancel_request_ready, Ordering::SeqCst);
        self.result_request_ready
            .store(result_request_ready, Ordering::SeqCst);
        self.goal_expired.store(goal_expired, Ordering::SeqCst);

        self.goal_request_ready.load(Ordering::SeqCst)
            || self.cancel_request_ready.load(Ordering::SeqCst)
            || self.result_request_ready.load(Ordering::SeqCst)
            || self.goal_expired.load(Ordering::SeqCst)
    }
}
