# publish_obstacles_node
This node provides the function to publish obstacles topics for navigation.

This node requires a [mapping_object_server](https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws/blob/review/src/01_common/hma_common/hma_common_pkg/docs/mapping_object_server.md).

## Interfaces
### Published topics
**obstacles ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))**

Object information is published as a point cloud for obstacles of navigation.

### Subscribed topics
**~run_enable ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))**

Control whether or not execution is performed.

If `True`, executed; if `False`, not executed.

### Action clients
**mapping_object ([hma_common_action/MappingObjectAction]())**

This is used to get objects information by mapping_object_server.

