use rosidl_runtime_rs::{Action, GetResultService, HasGoal, HasGoalId, Message, SendGoalService, Service};

use std::collections::HashMap;
use std::ffi::CString;
use std::sync::{atomic::AtomicBool, atomic::Ordering, Arc, Mutex, MutexGuard};

use crate::error::{RclActionReturnCode, RclReturnCode, RclrsError, ToResult};
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

#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct GoalUUID([u8; RCL_ACTION_UUID_SIZE]);

impl GoalUUID {
    pub fn new(uuid: [u8; RCL_ACTION_UUID_SIZE]) -> Self {
        Self(uuid)
    }
}

// type GetResultResponse<T> = <<T as rosidl_runtime_rs::Action>::GetResult as rosidl_runtime_rs::Service>::Response;
// type GetResultRequest<T> = <<T as rosidl_runtime_rs::Action>::GetResult as rosidl_runtime_rs::Service>::Request;

pub enum GoalResponse {
    Reject = 1,
    AcceptAndExecute = 2,
    AcceptAndDefer = 3,
}
pub enum CancelResponse {
    Reject = 1,
    Accept = 2,
}

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

pub struct ServerGoalHandle<T>
where
    T: rosidl_runtime_rs::Action,
{
    handle: Arc<ServerGoalHandleHandle>,
    goal: Arc<<T as Action>::Goal>,
    on_terminal_state: Box<dyn Fn(GoalUUID, Arc<<T::GetResult as GetResultService>::Response>) -> Result<(), RclrsError>>,
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
        goal: Arc<<T as Action>::Goal>,
        on_terminal_state: Box<dyn Fn(GoalUUID, Arc<<T::GetResult as GetResultService>::Response>) -> Result<(), RclrsError>>,
        on_executing: Box<dyn Fn(GoalUUID) -> Result<(), RclrsError>>,
        publish_feedback_cb: Box<dyn Fn(T::Feedback) -> Result<(), RclrsError>>,
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
        let mut state = &mut GoalStatus::STATUS_UNKNOWN;
        unsafe { rcl_action_goal_handle_get_status(*goal_handle, &mut *state) }.ok()?;
        Ok(*state)
    }

    pub fn get_goal(&self) -> Arc<T::Goal> {
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
    pub(crate) goal_handles_mtx: Arc<Mutex<HashMap<crate::action::GoalUUID, Arc<ServerGoalHandle<T>>>>>,
    pub(crate) goal_results: Arc<Mutex<HashMap<crate::action::GoalUUID, Arc<<T::GetResult as GetResultService>::Response>>>>,
    pub(crate) result_requests: Arc<Mutex<HashMap<crate::action::GoalUUID, Vec<rmw_request_id_t>>>>,
    pub(crate) handle: Arc<ActionServerHandle>,
    handle_goal_cb: fn(&crate::action::GoalUUID, Arc<<T as Action>::Goal>) -> GoalResponse,
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
        let server_clock = &mut *clock.lock();
        unsafe {
            // SAFETY: The rcl_action_server is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the subscription.
            // The topic name and the options are copied by this function, so they can be dropped
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

        let handle = Arc::new(ActionServerHandle {
            rcl_action_server_mtx: Mutex::new(rcl_action_server),
            rcl_node_mtx,
            in_use_by_wait_set: Arc::new(AtomicBool::new(false))
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

    
    pub fn take_goal_request(&self) -> Result<(<<T as Action>::SendGoal as SendGoalService>::Request, rmw_request_id_t), RclrsError> {
        let mut request_id_out = rmw_request_id_t {
            writer_guid: [0; 16],
            sequence_number: 0,
        };
        type RmwMsg<T> =
            <<<T as Action>::SendGoal as SendGoalService>::Request as Message>::RmwMsg;
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
        Ok((<<<T as Action>::SendGoal as SendGoalService>::Request>::from_rmw_message(request_out), request_id_out))
    }


pub fn take_result_request(&self) -> Result<(<T::GetResult as GetResultService>::Request, rmw_request_id_t), RclrsError> {
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
        Ok((<T::GetResult as GetResultService>::Request::from_rmw_message(request_out), request_id_out))
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

        
        let goal = Arc::new(goal_request.get_goal::<T>());
        let uuid = GoalUUID::new(goal_request.get_goal_id());
        let user_response = (self.handle_goal_cb)(&uuid, goal);

        type RmwMsg<T> = <<<T as Action>::SendGoal as Service>::Response as rosidl_runtime_rs::Message>::RmwMsg;
        let goal_response = RmwMsg::<T>::default();
        //TODO: Set the SendGoalResponse.accepted field to GoalResponse::ACCEPT_AND_EXECUTE == user_response || GoalResponse::ACCEPT_AND_DEFER == user_response;
        // per rclcpp
        let handle = &*self.handle.lock();
        unsafe {
            // SAFETY: The response type is guaranteed to match the service type by the type system.
            rcl_action_send_goal_response(
                handle,
                &mut req_id,
                &goal_response as *const RmwMsg<T> as *mut _,
            )
        }
        .ok()?;

        if matches!(user_response, GoalResponse::AcceptAndExecute | GoalResponse::AcceptAndDefer) {
            let goal_info_handle = GoalInfoHandle::new(self.handle.clone());
            let uuid = GoalUUID::new(goal_request.get_goal_id());
            goal_info_handle.lock().goal_id.uuid.copy_from_slice(&uuid.0);
            
            let goal_handle_handle = self.accept_new_goal(goal_info_handle);
            // let mut_handle = &mut *self.handle.lock();
            // let goal_info = &*goal_info_handle.rcl_action_goal_info_mtx.lock().unwrap();
            // unsafe { 
            //     let goal_handle_raw_ptr = rcl_action_accept_new_goal(mut_handle, goal_info);

            //     let goal_handle_handle = ServerGoalHandleHandle {
            //         rcl_goal_handle_mtx: Mutex::new(*goal_handle_raw_ptr)
            //     };
            // }
            
            

            // TODO: rclcpp also inserts into a map of GoalUUID and rcl_action_goal_handle_t pairs for some reason. Unsure if this is needed
            if matches!(user_response, GoalResponse::AcceptAndExecute) {
                let goal_handle = goal_handle_handle.lock();
                unsafe {
                    rcl_action_update_goal_state(*goal_handle, rcl_action_goal_event_e::GOAL_EVENT_EXECUTE)
                }
                .ok()?;
            }
            self.publish_status()?;
            self.call_goal_accepted_cb(goal_handle_handle, uuid, goal_request);
        }
        Ok(())
    }

    pub fn accept_new_goal(&self, goal_info_handle: GoalInfoHandle) -> ServerGoalHandleHandle {
        let mut_handle = &mut *self.handle.lock();
            let goal_info = &*goal_info_handle.rcl_action_goal_info_mtx.lock().unwrap();
            unsafe { 
                let goal_handle_raw_ptr = rcl_action_accept_new_goal(mut_handle, goal_info);
                ServerGoalHandleHandle::new(
                    Mutex::new(goal_handle_raw_ptr)
                )
            }
    }

    pub fn call_goal_accepted_cb(
        &self,
        goal_handle_handle: ServerGoalHandleHandle,
        goal_uuid: GoalUUID,
        goal_request: <<T as Action>::SendGoal as SendGoalService>::Request,
    ) -> () {
        let on_terminal_state = |uuid: GoalUUID, result: Arc<<T::GetResult as GetResultService>::Response>| -> Result<(), RclrsError> {
            self.publish_result(&uuid, result);
            self.publish_status();
            self.notify_goal_terminal_state();
            { 
                let mut goal_handles = self.goal_handles_mtx.lock().unwrap();
                goal_handles.remove(&uuid);
            }
            Ok(())
        };

        let on_executing = |uuid: GoalUUID| -> Result<(), RclrsError> {
            self.publish_status()
        };

        let publish_feedback = |feedback_msg: T::Feedback| -> Result<(), RclrsError> {
            self.publish_feedback(feedback_msg)
        };

        let goal_handle_arc = Arc::new(ServerGoalHandle::<T>::new(
            Arc::new(goal_handle_handle),
            Arc::new(goal_request.get_goal::<T>()),
            Box::new(on_terminal_state),
            Box::new(on_executing),
            Box::new(publish_feedback),
        ));
        { 
            let mut goal_handles = self.goal_handles_mtx.lock().unwrap();
            goal_handles.insert(goal_uuid, goal_handle_arc.clone());
        }
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
        let cancel_request_handle = CancelRequestHandle::new();
        let cancel_response_handle = CancelResponseHandle::new();
        let handle = &*self.handle.lock();
        let request_handle = &*cancel_request_handle.lock();
        let response_handle = &mut *cancel_response_handle.lock();
        unsafe {
            rcl_action_process_cancel_request(
                handle, 
                request_handle, 
                response_handle as *mut _
            )
        }
        .ok()?;
        let response_mtx = Arc::new(Mutex::new(CancelGoal_Response::default()));
        {
            let mut response = response_mtx.lock().unwrap();
            response.return_code = response_handle.msg.return_code;
        }
        let goals = unsafe { std::slice::from_raw_parts(response_handle.msg.goals_canceling.data, response_handle.msg.goals_canceling.size) };

        for i in 0..goals.len() {
            let goal_info = &goals[i];
            let mut uuid = GoalUUID::new([0; RCL_ACTION_UUID_SIZE]);
            uuid.0.copy_from_slice(&goal_info.goal_id.uuid);
            let cb_response_code = self.call_handle_cancel_callback(uuid);
            if matches!(CancelResponse::Accept, cb_response_code) {
                let mut rs_goal_info = GoalInfo::default();
                rs_goal_info.goal_id.uuid.clone_from_slice(&goal_info.goal_id.uuid);
                //TODO: Set the timestamp for the rs_goal_info
                { 
                    let mut response = response_mtx.lock().unwrap();
                    response.goals_canceling.push(rs_goal_info);
                }
            }
        }

        // If the user rejects all individual requests to cancel goals,
        // then we consider the top-level cancel request as rejected.
        {
            let mut response = response_mtx.lock().unwrap();
            if goals.len() >= 1 && 0 == response.goals_canceling.len() {
                response.return_code = CancelResponse::Reject as i8;
            }
            
            // Probably overkill
            if !response.goals_canceling.is_empty() {
                // at least one goal state changed, publish a new status message
                self.publish_status();
            }
        }

        {
            let response = response_mtx.lock().unwrap();
            let response_cow = response.clone().into_cow();
            let rmw_message = <CancelGoal_Response as Message>::into_rmw_message(response_cow);
            unsafe {
                // SAFETY: The response type is guaranteed to match the service type by the type system.
                rcl_action_send_cancel_response(
                    handle,
                    &mut req_id,
                    rmw_message.as_ref() as *const <CancelGoal_Response as Message>::RmwMsg as *mut _,
                )
            }
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
        // let goal_uuid = result_request.uuid;
        let goal_uuid = GoalUUID::new(result_request.get_goal_id());
        let goal_info_handle = GoalInfoHandle::new(self.handle.clone());

        let handle = &*self.handle.lock();
        let goal_exists = unsafe { rcl_action_server_goal_exists(handle, &*goal_info_handle.lock()) };
        type RmwMsg<T> = <<<T as Action>::GetResult as GetResultService>::Response as Message>::RmwMsg;
        if !goal_exists {
            //TODO: set this to unknown
            let mut result_response = RmwMsg::<T>::default();
            unsafe {
                rcl_action_send_result_response(handle, &mut req_id, &mut result_response as *mut RmwMsg<T> as *mut _)
            }
            .ok()?;
        } else {
            if let Some(result_response) = { self.goal_results.lock().unwrap() }.remove(&goal_uuid)
            {
                match Arc::into_inner(result_response) {
                    Some(res) => {
                        let rmw_message = <<<T as Action>::GetResult as GetResultService>::Response as Message>::into_rmw_message(res.into_cow());
                        unsafe {
                            rcl_action_send_result_response(handle, &mut req_id, rmw_message.as_ref() as *const RmwMsg<T> as *mut _)
                        }
                        .ok()?;
                        let sent_response = <<<T as Action>::GetResult as GetResultService>::Response as Message>::from_rmw_message(rmw_message.into_owned());
                        { self.goal_results.lock().unwrap() }.insert(goal_uuid, Arc::new(sent_response));
                    }
                    None => {}
                }
            } else {
                if let Some(request_headers) = { self.result_requests.lock().unwrap() }.get_mut(&goal_uuid)
                {
                    request_headers.push(req_id);
                }
            }
        }
        Ok(())
    }

    pub fn execute_check_expired_goals(&self) -> Result<(), RclrsError> {
        let goal_info_handle = GoalInfoHandle::new(self.handle.clone());
        let mut num_expired: usize = 1;
        while num_expired > 0 {
            let handle = &*self.handle.lock();
            unsafe { rcl_action_expire_goals(handle, &mut *goal_info_handle.lock(), 1, &mut num_expired) }.ok()?;
            if num_expired > 0 {
                // TODO: Convert the expired goal_info uuid to a uuid
                let mut goal_uuid = GoalUUID::new([0; RCL_ACTION_UUID_SIZE]);
                goal_uuid.0.copy_from_slice(&goal_info_handle.lock().goal_id.uuid);
                { self.goal_results.lock().unwrap() }.remove(&goal_uuid);
                { self.result_requests.lock().unwrap() }.remove(&goal_uuid);
                { self.goal_handles_mtx.lock().unwrap() }.remove(&goal_uuid);
            }
        }
        Ok(())
    }

    pub fn publish_status(&self) -> Result<(), RclrsError> {
        let mut num_goals: usize = 0;
        let goal_handles = std::ptr::null_mut();
        let handle = &*self.handle.lock();
        unsafe { rcl_action_server_get_goal_handles(handle, goal_handles, &mut num_goals) }.ok()?;
        let mut goal_status_array = GoalStatusArray::default();
        goal_status_array.status_list.reserve(num_goals);
        let mut c_status_array = unsafe { rcl_action_get_zero_initialized_goal_status_array() };
        unsafe { rcl_action_get_goal_status_array(handle, &mut c_status_array) }.ok()?;
        unsafe { rcl_action_goal_status_array_fini(&mut c_status_array) }.ok()?;
        let status_array_slice = unsafe { std::slice::from_raw_parts( c_status_array.msg.status_list.data, c_status_array.msg.status_list.size) };
        for i in 0..status_array_slice.len() {
            let status_msg = &status_array_slice[i];

            let mut goal_status = GoalStatus::default();
            goal_status.status = status_msg.status;
            goal_status.goal_info.stamp = crate::vendor::builtin_interfaces::msg::Time { 
                sec: status_msg.goal_info.stamp.sec, 
                nanosec: status_msg.goal_info.stamp.nanosec 
            };
            goal_status.goal_info.goal_id.uuid.copy_from_slice(&status_msg.goal_info.goal_id.uuid);
            goal_status_array.status_list.push(goal_status);
        }
        let goal_status_array_rmw_msg = <GoalStatusArray as rosidl_runtime_rs::Message>::into_rmw_message(goal_status_array.into_cow());
        unsafe { 
            rcl_action_publish_status(
                handle, 
                goal_status_array_rmw_msg.as_ref() as *const <GoalStatusArray as Message>::RmwMsg as *mut _
            ) 
        }.ok()?;
        Ok(())
    }

    pub fn publish_result(&self, goal_uuid: &GoalUUID, result: Arc<<T::GetResult as GetResultService>::Response>) -> Result<(), RclrsError> {
        let goal_info = GoalInfoHandle::new(self.handle.clone());
        goal_info.lock().goal_id.uuid.copy_from_slice(&goal_uuid.0);
        let server_handle = &*self.handle.lock();
        let goal_info_handle = &*goal_info.lock();
        let goal_exists = unsafe { rcl_action_server_goal_exists(server_handle, goal_info_handle) };
        if !goal_exists {
            panic!("Asked to publish result for goal that does not exist");
        }
        { self.goal_results.lock().unwrap() }.insert(goal_uuid.clone(), result.clone());
        match Arc::into_inner(result) {
            Some(res) => {
                let result_rmw_message = <<<T as Action>::GetResult as GetResultService>::Response as Message>::into_rmw_message(res.into_cow());
                if let Some(req_ids) = { self.result_requests.lock().unwrap() }.get_mut(&goal_uuid) {
                    type RmwMsg<T> = <<<T as Action>::GetResult as GetResultService>::Response as Message>::RmwMsg;
                    for req_id in req_ids {
                        unsafe {
                            rcl_action_send_result_response(
                                server_handle, 
                                req_id as *mut _, 
                                result_rmw_message.as_ref() as *const RmwMsg<T> as *mut _,
                            )
                        }.ok()?;
                    }
                }
            }
            None => { 
                return Err(RclrsError::RclError { code: crate::RclReturnCode::Error, msg: None });
            }
        }
        
        Ok(())
    }

    pub fn notify_goal_terminal_state(&self) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_notify_goal_done(handle)
        }.ok()?;
        Ok(())
    }

    pub fn call_handle_cancel_callback(&self, goal_uuid: GoalUUID) -> CancelResponse {
        let binding = { self.goal_handles_mtx.lock().unwrap() };
        let goal_handle_option = binding.get(&goal_uuid);
        match goal_handle_option {
            Some(handle) => {
                let cancel_cb_response = (self.handle_cancel_cb)(handle.clone());
                match cancel_cb_response {
                    CancelResponse::Accept => {
                        let result = handle._cancel_goal();
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

pub struct GoalInfoHandle {
    rcl_action_goal_info_mtx: Mutex<rcl_action_goal_info_t>,
    rcl_action_server: Arc<ActionServerHandle>,
}

impl GoalInfoHandle {
    pub fn new(rcl_action_server: Arc<ActionServerHandle>) -> Self {
        let goal_info = unsafe { rcl_action_get_zero_initialized_goal_info() };
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

    // /// Sets the goal request ready state to value
    // fn set_goal_request_ready(&self, value: bool) -> ();
    // /// Sets the cancel request ready state to value
    // fn set_cancel_request_ready(&self, value: bool) -> ();
    // /// Sets the result request ready state to value
    // fn set_result_request_ready(&self, value: bool) -> ();
    // /// Sets the goal expired state to value
    // fn set_goal_expired(&self, value: bool) -> ();

    fn is_ready(&self, wait_set: &mut rcl_wait_set_t) -> bool;
}

impl<T> ActionServerBase for ActionServer<T> 
where T: rosidl_runtime_rs::Action {

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
                &mut goal_expired
            )
        };

        self.goal_request_ready.store(goal_request_ready, Ordering::SeqCst);
        self.cancel_request_ready.store(cancel_request_ready, Ordering::SeqCst);
        self.result_request_ready.store(result_request_ready, Ordering::SeqCst);
        self.goal_expired.store(goal_expired, Ordering::SeqCst);
        
        self.goal_request_ready.load(Ordering::SeqCst)
            || self.cancel_request_ready.load(Ordering::SeqCst)
            || self.result_request_ready.load(Ordering::SeqCst)
            || self.goal_expired.load(Ordering::SeqCst)
    }
}

pub struct CancelRequestHandle {
    rcl_action_cancel_request_mtx: Mutex<rcl_action_cancel_request_t>
}

impl CancelRequestHandle {
pub fn new() -> Self {
        let rcl_action_cancel_req = unsafe { rcl_action_get_zero_initialized_cancel_request() };
        Self {
            rcl_action_cancel_request_mtx: Mutex::new(rcl_action_cancel_req)
        }
    }

    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_cancel_request_t> {
        self.rcl_action_cancel_request_mtx.lock().unwrap()
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
        self.rcl_action_cancel_response_mtx.lock().unwrap()
    }
}

impl Drop for CancelResponseHandle {
    fn drop(&mut self) {
        let cancel_response = &mut *self.rcl_action_cancel_response_mtx.get_mut().unwrap();
        unsafe {
            rcl_action_cancel_response_fini(cancel_response);
        }
    }
}
