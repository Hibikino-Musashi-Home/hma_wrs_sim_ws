# grasp_pose_estimation_server
This action server provides the function to perform grasp pose estimation for the specific objects.

## Interfaces
### Parameters
**~camera_info (str)**

Specify the topic name to subscribe to the camera information.

Defaults to `"/camera/depth_registered/camera_info"`.

**~size_th/min/height (float)**

If the object's height is less than the specified value, it is grasped from the top.

If the object's height is greater than the specified value, it is grasped from the top or the front with reference `~size_th/max/width.`

Defaults to `0.15` [m].

**~size_th/max/width (float)**

If the object's width is less than the specified value, it is grasped from the front.

If the object's width is greater than the specified value, it is grasped from the top.

Defaults to `0.14` [m].

### Published topics
**~dbg ([sensor_msgs/PointCloud2.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))**

Publish a 3D point cloud of grasping objects for visualization.

**~marker ([visualization_msgs/Marker.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))**

Publish a rectangular marker for visualization.

The color of markers are blue when grasped from top and red when grasped from the front.

**camera/depth_registered/camera_info ([sensor_msgs/CameraInfo.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html))**

Subscribe to the camera information.

Subscribe only once for the first time.

The topic name is changed by the `~camera_info` parameter.

### Action servers
**grasp_pose_estimation ([hma_common_action/GraspPoseEstimationAction]())**

This action server provides a function to estimate the grasping pose of a specific object.