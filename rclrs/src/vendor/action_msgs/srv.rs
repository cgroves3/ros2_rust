#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

//TODO: Fill this stuff in
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelGoal_Request {
    pub goal_info: crate::vendor::action_msgs::msg::GoalInfo
}

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelGoal_Response {
    pub return_code: u8,
    pub goals_canceling: Vec<crate::vendor::action_msgs::msg::GoalInfo>
}
