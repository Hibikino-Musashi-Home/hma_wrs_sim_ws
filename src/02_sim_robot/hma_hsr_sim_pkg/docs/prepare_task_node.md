# prepare_task_node
This node provides the function to prepare the task environments using [tmc_wrs_gazebo_worlds/spawn_objects](https://github.com/hsr-project/tmc_wrs_gazebo/blob/master/tmc_wrs_gazebo_worlds/scripts/spawn_objects).

## Quick start
Please start the WRS Simulator before inputting the following command.
```
$ roslaunch hma_hsr_sim_pkg prepare_task.launch
```
You can see the deleting objects and spawning objects on the simulator.

## Usage
The following description in the launch file will automatically prepare the task environment with the specified settings.
```
<launch>
    ...
    <arg name="prep" default="true"/>
    <arg name="seed" default="-1"/>
    <arg name="percategory" default="5"/>
    <arg name="obstacles" default="3"/>
    <arg name="perrow" default="4"/>
    <arg name="drawer_open" default="false"/>
    <arg name="task" default="task"/>
    <node pkg="hma_hsr_sim_pkg" type="prepare_task_node.py" name="prepare_task_node" output="screen">
        <param name="/prep" type="bool" value="$(arg prep)"/>
        <param name="/seed" type="int" value="$(arg seed)"/>
        <param name="/percategory" type="int" value="$(arg percategory)"/>
        <param name="/obstacles" type="int" value="$(arg obstacles)"/>
        <param name="/perrow" type="int" value="$(arg perrow)"/>
        <param name="/drawer_open" type="bool" value="$(arg drawer_open)"/>
        <param name="/task" type="str" value="$(arg task)"/>
    </node>
    ...
</launch>
```

## Interfaces
### Parameters
**~prep (bool)**

Whether to prepare for the task.

Defaults to `True`.

**~seed (int)**

Specify the seed value.

If not specified (negative value), it is a random seed (0 to 10000).

Defaults to `-1`.

**~percategory (int)**

Number of objects in each category in Task 1.

Defaults to `5`.

**~obstacles (int)**

Number of obstacles objects in Task 2a.

Defaults to `3`.

**~perrow (int)**

Number of objects on each tier of the shelf in Task 2b.

Defaults to `3`.

**~drawer_open (bool)**

Whether drawers are opened automatically or not.

If `True`, the three drawers open automatically after 30 seconds of startup.

Defaults to `False`.

**~delete_prefix (str)**

Specify a prefix to delete an object from the simulator.

`task`, `task1`, `task2`, `task2a`, and `task2b` can be specified.

For example, in the case of `task`, all objects are deleted.
In the case of `task1`, only the objects in Task 1 are deleted.

Defaults to `task`.

### Subscribed topics
**gazebo/model_states ([gazebo_msgs/ModelStates.msg](http://docs.ros.org/en/jade/api/gazebo_msgs/html/msg/ModelStates.html))**

Get the states of a model existing on Gazebo.