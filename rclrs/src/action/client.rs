// use crate::action::types::GoalUUID;
use crate::error::{RclrsError, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;

use std::sync::atomic::{AtomicBool, Ordering};
use std::collections::HashMap;
use std::ffi::CString;
use std::sync::{Arc, Mutex, MutexGuard};


// SAFETY: The functions accessing this type, including drop(), shouldn't care about the thread
// they are running in. Therefore, this type can be safely sent to another thread.
unsafe impl Send for rcl_action_client_t {}

/// Internal struct used by action clients.
#[derive(Debug)]
pub struct ActionClientHandle {
    rcl_action_client_mtx: Mutex<rcl_action_client_t>,
    rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
}

impl ActionClientHandle {
    /// Creates a new action client handle
    pub(crate) fn lock(&self) -> MutexGuard<rcl_action_client_t> {
        self.rcl_action_client_mtx.lock().unwrap()
    }

    /// Returns if the any requests are ready
    pub fn action_server_is_ready(&self) -> Result<bool, RclrsError> {
        let mut is_ready = false;
        let handle = &*self.rcl_action_client_mtx.lock().unwrap();
        let node_handle = &*self.rcl_node_mtx.lock().unwrap();
        unsafe {
            rcl_action_server_is_available(
                node_handle,
                handle,
                &mut is_ready
            ).ok()?;
        }
        Ok(is_ready)
    }
}

impl Drop for ActionClientHandle {
    fn drop(&mut self) {
        let rcl_action_client = self.rcl_action_client_mtx.get_mut().unwrap();
        let rcl_node = &mut *self.rcl_node_mtx.lock().unwrap();
        // SAFETY: No preconditions for this function (besides the arguments being valid).
        unsafe {
            rcl_action_client_fini(rcl_action_client, rcl_node);
        }
    }
}

/// The struct representing the ROS Action Client
pub struct ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    handle: Arc<ActionClientHandle>,
    feedback_ready: Arc<AtomicBool>,
    status_ready: Arc<AtomicBool>,
    goal_response_ready: Arc<AtomicBool>,
    cancel_response_ready: Arc<AtomicBool>,
    result_response_ready: Arc<AtomicBool>,
    num_subscriptions: usize,
    num_guard_conditions: usize,
    num_timers: usize,
    num_clients: usize,
    num_services: usize,
    /// rclcpp uses a std::map, which is ordered, but it doesn't get the minimum/maximum key, so instead a hashmap is used
    pending_goal_responses: Arc<Mutex<HashMap<i64, T::Goal>>>,
    /// rclcpp uses a std::map, which is ordered, but it doesn't get the minimum/maximum key, so instead a hashmap is used
    pending_result_responses: Arc<Mutex<HashMap<i64, T::Result>>>,
    /// rclcpp uses a std::map, which is ordered, but it doesn't get the minimum/maximum key, so instead a hashmap is used
    pending_cancel_responses: Arc<Mutex<HashMap<i64, crate::vendor::action_msgs::srv::CancelGoal>>>
    //TODO: In rclcpp, there is a few members I'm not sure what they are for:
    //   rclcpp::Context::SharedPtr context_;
    //   rclcpp::node_interfaces::NodeGraphInterface::WeakPtr node_graph_;
}

impl<T> ActionClient<T>
where
    T: rosidl_runtime_rs::Action,
{
    /// Creates a new action client.
    pub(crate) fn new(
        rcl_node_mtx: Arc<Mutex<rcl_node_t>>,
        name: &str,
        qos: QoSProfile,
        ) -> Result<Self, RclrsError>
    where
        T: rosidl_runtime_rs::Action,
    {
        // SAFETY: Getting a zero-initialized value is always safe.
        let mut rcl_action_client = unsafe { rcl_action_get_zero_initialized_client() };
        let type_support_ptr = 
            <T as rosidl_runtime_rs::Action>::get_type_support() as *const rosidl_action_type_support_t;
        let name_c_string = CString::new(name).map_err(|err| RclrsError::StringContainsNul {
            err,
            s: name.into(),
        })?;    
        // SAFETY: No preconditions for this function.
        let mut client_options = unsafe { rcl_action_client_get_default_options() };
        client_options.goal_service_qos = qos.into();
        client_options.cancel_service_qos = qos.into();
        client_options.result_service_qos = qos.into();
        client_options.feedback_topic_qos = qos.into();
        client_options.status_topic_qos = qos.into();
        unsafe { 
            // SAFETY: The rcl_action_client is zero-initialized as expected by this function.
            // The rcl_node is kept alive because it is co-owned by the action server.
            // The action name and the options are copied by this function, so they can be dropped
            // afterwards.
            rcl_action_client_init(
                &mut rcl_action_client,
                &mut *rcl_node_mtx.lock().unwrap(),
                type_support_ptr,
                name_c_string.as_ptr(),
                &client_options
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
                &rcl_action_client,
                &mut num_subscriptions,
                &mut num_guard_conditions,
                &mut num_timers,
                &mut num_clients,
                &mut num_services
            ).ok()?;
        }
        
        let handle = Arc::new(
            ActionClientHandle {
                rcl_action_client_mtx: Mutex::new(rcl_action_client),
                rcl_node_mtx: rcl_node_mtx,
            }
        );

        Ok(Self {
            handle: handle,
            feedback_ready: Arc::new(AtomicBool::new(false)),
            status_ready: Arc::new(AtomicBool::new(false)),
            goal_response_ready: Arc::new(AtomicBool::new(false)),
            cancel_response_ready: Arc::new(AtomicBool::new(false)),
            result_response_ready: Arc::new(AtomicBool::new(false)),
            num_subscriptions: num_subscriptions,
            num_guard_conditions: num_guard_conditions,
            num_timers: num_timers,
            num_clients: num_clients,
            num_services: num_services,
            pending_goal_responses: Arc::new(Mutex::new(HashMap::new())),
            pending_result_responses: Arc::new(Mutex::new(HashMap::new())),
            pending_cancel_responses: Arc::new(Mutex::new(HashMap::new()))
        })
    }
}

/// Trait to be implemented by concrete Action Client structs.
///
/// See [`ActionClientT>`] for an example
pub trait ActionClientBase: Send + Sync {

    /// Return true if there is an action server that is ready to take goal requests.
    fn action_server_is_ready(&self) -> Result<bool, RclrsError>;

    /// Returns true if any feedback, status, goal response, cancel response and result response is ready
    fn is_ready(&self, wait_set: &rcl_wait_set_t) -> Result<bool, RclrsError>;

    /// The number of ready subscriptions
    fn get_number_of_ready_subscriptions(&self) -> usize;

    /// The number of ready guard conditions
    fn get_number_of_ready_guard_conditions(&self) -> usize;

    /// The number of ready timers
    fn get_number_of_ready_timers(&self) -> usize;

    /// The number of ready client
    fn get_number_of_ready_clients(&self) -> usize;

    /// The number of ready services
    fn get_number_of_ready_services(&self) -> usize;

    /// Adds the action client to the wait set
    fn add_to_wait_set(&self, wait_set: &mut rcl_wait_set_t) -> Result<(), RclrsError>;
}

impl<T> ActionClientBase for ActionClient<T> 
where
    T: rosidl_runtime_rs::Action
{
    fn action_server_is_ready(&self) -> Result<bool, RclrsError> {
        self.handle.action_server_is_ready()
    }

    fn is_ready(&self, wait_set: &rcl_wait_set_t) -> Result<bool, RclrsError> {
        
        let mut feedback_ready = false;
        let mut status_ready = false;
        let mut goal_response_ready = false;
        let mut cancel_response_ready = false;
        let mut result_response_ready = false;

        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_client_wait_set_get_entities_ready(
                wait_set as *const _,
                handle as *const _,
                &mut feedback_ready,
                &mut status_ready,
                &mut goal_response_ready,
                &mut cancel_response_ready,
                &mut result_response_ready
            ).ok()?
        }

        self.feedback_ready.store(feedback_ready, Ordering::SeqCst);
        self.status_ready.store(status_ready, Ordering::SeqCst);
        self.goal_response_ready.store(goal_response_ready, Ordering::SeqCst);
        self.cancel_response_ready.store(cancel_response_ready, Ordering::SeqCst);
        self.result_response_ready.store(status_ready, Ordering::SeqCst);

        Ok(
            self.feedback_ready.load(Ordering::SeqCst) ||
            self.status_ready.load(Ordering::SeqCst) ||
            self.goal_response_ready.load(Ordering::SeqCst) ||
            self.cancel_response_ready.load(Ordering::SeqCst) ||
            self.result_response_ready.load(Ordering::SeqCst)
        )
    }

    fn get_number_of_ready_subscriptions(&self) -> usize {
        self.num_subscriptions
    }

    fn get_number_of_ready_guard_conditions(&self) -> usize {
        self.num_guard_conditions
    }

    fn get_number_of_ready_timers(&self) -> usize {
        self.num_timers
    }

    fn get_number_of_ready_clients(&self) -> usize {
        self.num_clients
    }

    fn get_number_of_ready_services(&self) -> usize {
        self.num_services
    }
    
    fn add_to_wait_set(&self, wait_set: &mut rcl_wait_set_t) -> Result<(), RclrsError> {
        let handle = &*self.handle.lock();
        unsafe {
            rcl_action_wait_set_add_action_client(
                wait_set as *mut _,
                handle,
                std::ptr::null_mut(),
                std::ptr::null_mut()
            ).ok()?;
        }
        Ok(())
    }
}