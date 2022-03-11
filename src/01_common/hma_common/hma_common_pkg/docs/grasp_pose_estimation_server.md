# grasp_pose_estimation_server
This action server provides the function to perform grasp pose estimation for the specific objects.

## Interfaces
### Parameters
**~camera_info (str)**

Specify the topic name to subscribe to the camera information.

Defaults to `/camera/depth_registered/camera_info`.

**~size_th/min/height (float)**

If the object's height is less than the specified value, it is grasped from the top.

If the object's height is greater than the specified value, it is grasped from the top or the front with reference `~size_th/max/width.`

Defaults to `0.15`.

**~size_th/max/width (float)**

If the object's width is less than the specified value, it is grasped from the front.

If the object's width is greater than the specified value, it is grasped from the top.

Defaults to `0.14`.

### Published topics
**dbg ([sensor_msgs/PointCloud2.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))**

Publish a 3D point cloud of grasping objects for visualization.

**marker ([visualization_msgs/Marker.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))**

Publish a rectangular marker for visualization.

The color of markers are blue when grasped from top and red when grasped from the front.

### Action servers
**grasp_pose_estimation ([hma_common_action/GraspPoseEstimationAction](https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws/blob/review_/hma_common/src/01_common/hma_common/hma_common_action/action/GraspPoseEstimation.action))**

This action server provides a function to estimate the grasping pose of a specific object.

The required arguments and return values for the action are as follows.
- Args
    - conv_frame (str): Name of the converted frame.
    - origin_frame (str): Name of the original frame.
    - id (str): Object ID.
    - d ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)): Depth image.
    - mask ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)): Mask image of the object to grasp pose estimation.
    - specific_direction (int): Fix the grasping direction to be estimated. Select from GraspPose.[TOP, FRONT].
        - TOP: Estimate the grasp pose only from the top.
        - FRONT: Estimate the grasp pose only from the front.

- Returns:
    - [hma_common_action/GraspPoseEstimationResult](https://github.com/Hibikino-Musashi-Home/hma_wrs_sim_ws/blob/review_/hma_common/src/01_common/hma_common/hma_common_action/action/GraspPoseEstimation.action): Results of grasp pose estimation.
