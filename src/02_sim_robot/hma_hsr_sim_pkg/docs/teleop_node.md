# prepare_task_node
This node provides the function to prepare the task environments using [tmc_wrs_gazebo_worlds/spawn_objects](https://github.com/hsr-project/tmc_wrs_gazebo/blob/master/tmc_wrs_gazebo_worlds/scripts/spawn_objects).

## Quick start
Please start the WRS Simulator before inputting the following command.
```
$ roslaunch hma_hsr_sim_pkg teleop.launch
```
The window (Teleop) created by pygame is displayed.

You can operate HSR by pressing `WASDQE` on your keyboard on the window.

`W`: Forward, `S`: Backward, `A`: Left, `D`: Right, `Q`: Rotate left, `E`: Rotate right.

## Interfaces
### Subscribed topics
**teleop_node/run_enable ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**

Control whether or not execution is performed.

If `True`, executed; if `False`, not executed.