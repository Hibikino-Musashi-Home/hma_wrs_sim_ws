# omni_path_follow_server
This node provides the function of path following for navigation.

## Interfaces
### Published topics
**path_follow_action/goal (tmc_omni_path_follower/PathFollowerActionGoal)**

Execute tmc_omni_path_follower.

**path_follow_action/cancel ([actionlib_msgs/GoalStatus](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html))**

Cancel tmc_omni_path_follower.

**~goal_status ([actionlib_msgs/GoalID](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalID.html))**

Publish status of path following.

### Action servers
**omni_path_follow ([hma_hsr_nav_action/OmniPathFollower]())**

This action server provides the function for path following.