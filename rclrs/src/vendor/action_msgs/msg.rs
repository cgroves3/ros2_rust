
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GoalInfo {
    // TODO: finish this
    goal_id: unique_identifier_msgs__msg__UUID,
    stamp: crate::vendor::builtin_interfaces::msg::Time,
}