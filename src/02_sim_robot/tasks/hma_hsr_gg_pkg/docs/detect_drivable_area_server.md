# detect_drivable_area_server
This action server provides the function to detect the drivable area to avoid obstacles.

## Interfaces
### Parameters
**~d (str)**

Specify the topic name to subscribe to the depth image.

Defaults to `"/camera/depth_registered/image_raw"`.

**~camera_info (str)**

Specify the topic name to subscribe to the camera information.

Defaults to `"/camera/depth_registered/camera_info"`.

**~robot_radius (float)**

Set the radius of the robot.

Defaults to `0.25` [m].

### Published topics
**obstacles ([sensor_msgs/PointCloud2.msg](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))**

Publish obstacle information for navigation.

**map/hma ([nav_msgs/OccupancyGrid.msg](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html))**

Publish the environment map with obstacles added.

### Subscribed topics
**detect_drivable_area_server/finish ([std_msgs/Empty.msg](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Empty.html))**

Subscribe to the finish decision.

The obstacle information will continue to be published until this topic is subscribed.

**camera/depth_registered/image_raw ([sensor_msgs/Image.msg](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))**

Subscribe to the depth image.

The topic name is changed by the `~d` parameter.

**map/hma ([nav_msgs/OccupancyGrid.msg](http://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html))**

Subscribe to the environment map before obstacles are added.

Subscribe only once for the first time.

**camera/depth_registered/camera_info ([sensor_msgs/CameraInfo.msg](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html))**

Subscribe to the camera information.

Subscribe only once for the first time.

The topic name is changed by the `~camera_info` parameter.

### Action servers
**detect_drivable_area ([hma_hsr_gg_pkg/DetectDrivableAreaAction](https://hibikino-musashi-home.github.io/hma_wrs_sim_ws/src/02_sim_robot/tasks/hma_hsr_gg_pkg/action/DetectDrivableArea.html))**

This action server provides a function to detect the drivable area to aoivd obstacles.