mod client;
mod server;
mod types;

pub use client::*;
pub use server::*;

use crate::rcl_bindings::RCL_ACTION_UUID_SIZE;

/// The Goal UUID
#[derive(Debug, PartialEq, Eq, Hash, Clone)]
pub struct GoalUUID([u8; RCL_ACTION_UUID_SIZE]);

impl GoalUUID {
    /// Creates a new goal uuid
    pub fn new(uuid: [u8; RCL_ACTION_UUID_SIZE]) -> Self {
        Self(uuid)
    }
}
