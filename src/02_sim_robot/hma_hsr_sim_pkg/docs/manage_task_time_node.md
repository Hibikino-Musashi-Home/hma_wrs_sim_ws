# manage_task_time_node
This node provides the function to manage the task time.

## ROS node features
The following statement in the launch file will cause all programs to terminate when the specified **simulation time** has elapsed.
```
<launch>
    ...
    <arg name="time" default="14.0"/>
    <node pkg="hma_hsr_sim_pkg" type="manage_task_time_node.py" name="manage_task_time_node" output="screen" required="true">
        <param name="/time" type="double" value="$(arg time)"/>
    </node>
    ...
</launch>
```

## Interfaces
### Parameters
**~time (float)**

Set the task time in minutes.

When the set time elapsed, the node terminates.

### Subscribed topics
**manage_task_time_node/run_enable ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**

Control whether or not execution is performed.

If `True`, executed; if `False`, not executed.