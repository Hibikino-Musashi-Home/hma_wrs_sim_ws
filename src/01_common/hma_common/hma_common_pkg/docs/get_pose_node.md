# get_pose_node
This node provides the function to get the robot's position on the map.

## Quick start
Please start the WRS Simulator before inputting the following command.
```
$ roslaunch hma_hsr_sim_pkg get_pose.launch
```
You can see the (x, y, yaw) position on the terminal.

## Interfaces
### Subscribed topics
**~run_enable ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**

Control whether or not execution is performed.

If `True`, executed; if `False`, not executed.