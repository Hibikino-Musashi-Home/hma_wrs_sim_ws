# manage_cmd_vel_node
This node provides the function to manage the command_velocity of HSR.

## Interfaces
### Parameters
**~enable (bool)**

Set whether or not to manage.

**~linear_vel/min (float)**

Set the minimum linear velocity.

**~linear_vel/max (float)**

Set the maximum linear velocity.

**~angular_vel/min (float)**

Set the minimum angular velocity.

**~angular_vel/max (float)**

Set the maximum angular velocity.

### Published topics
**hsrb/command_velocity ([geometry_msgs/Twist](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html))**
Publish the command velocity of HSR.

### Subscribed topics
**manage_cmd_vel_node/run_enable ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**

Control whether or not execution is performed.

If `True`, executed; if `False`, not executed.

**hsrb/manage/command_velocity ([geometry_msgs/Twist](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html))**

Subscribe to intermediate topics for management.