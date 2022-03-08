#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2022 Hibikino-Musashi@Home
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Hibikino-Musashi@Home nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, mainUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import rospy
import actionlib
import numpy as np
import copy
import threading
import open3d
import dynamic_reconfigure.client

from scipy.spatial.transform import Rotation

from std_msgs.msg import  Header, Empty
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid

from hma_hsr_gg_pkg.msg import DetectDrivableAreaAction
from hma_hsr_gg_pkg.msg import DetectDrivableAreaResult

sys.path.remove("/opt/ros/melodic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge


# Globals
GP_LOOP_RATE = 1.0


class DetectDrivableAreaServer:
    """Detect drivable area ROS Action server."""
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.is_init = False

        self.smi_rgb = Image()
        self.smi_d = Image()
        self.cb = CvBridge()

        self.finish = False
        self.map = OccupancyGrid()

        # ROS I/F
        self.p_d = rospy.get_param(rospy.get_name() + "/d",
                                   "/camera/depth_registered/image_raw")
        self.p_camera_info = rospy.get_param(rospy.get_name() + "/camera_info",
                                          "/camera/depth_registered/camera_info")
        self.p_robot_radius = rospy.get_param(rospy.get_name() + "/robot_radius",
                                              0.25)

        self.pub_obstacle_points = rospy.Publisher("/obstacles",
                                                   PointCloud2,
                                                   queue_size = 1)
        self.pub_map = rospy.Publisher("/map/hma",
                                       OccupancyGrid,
                                       queue_size = 1)

        self.sub_finish = rospy.Subscriber(rospy.get_name() + "/finish",
                                           Empty,
                                           self.subf_finish,
                                           queue_size = 1)

        self.sub_d = rospy.Subscriber(self.p_d,
                                      Image,
                                      self.subf_depth,
                                      queue_size = 1)

        self.camera_info = rospy.wait_for_message(self.p_camera_info, CameraInfo)
        self.intrinsics = open3d.camera.PinholeCameraIntrinsic()
        self.intrinsics.set_intrinsics(width=self.camera_info.width,
                                       height=self.camera_info.height,
                                       fx=self.camera_info.K[0],
                                       fy=self.camera_info.K[4],
                                       cx=self.camera_info.K[2],
                                       cy=self.camera_info.K[5])

        self.map = rospy.wait_for_message("/map/hma", OccupancyGrid)

        self.as_detect_drivable_area = actionlib.SimpleActionServer(
            "/detect_drivable_area",
            DetectDrivableAreaAction,
            self.main)
        self.as_detect_drivable_area.start()

        self.client_global_costmap = dynamic_reconfigure.client.Client(
            "/hma_move_base/global_costmap", timeout=10)
        self.client_global_costmap_inflater = dynamic_reconfigure.client.Client(
            "/hma_move_base/global_costmap/inflater", timeout=10)
        self.client_local_costmap = dynamic_reconfigure.client.Client(
            "/hma_move_base/local_costmap", timeout=10)
        self.client_local_costmap_inflater = dynamic_reconfigure.client.Client(
            "/hma_move_base/local_costmap/inflater", timeout=10)

        self.is_init = True

        return

    def delete(self):
        return

    def subf_finish(self, finish):
        """Callback function for finish signal."""
        self.lock.acquire()
        self.finish = True
        self.lock.release()

        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return

    def subf_depth(self, smi_d):
        """Callback function for depth image.
        
        Args:
            smi_d (sensor_msgs/Image): Depth image.
        """
        self.lock.acquire()
        self.smi_d = smi_d
        self.lock.release()

        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return


    def smi2cv(self, smi, encode, is_depth=False):
        """Convert from sensor_msgs/Image to OpenCV image.

        Args:
            smi (sensor_msgs/Image): Image before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.
            is_depth (bool, optional): Whether to convert depth image. Defaults to False.

        Returns:
            ndarray: Converted OpenCV image.
            If False is returned, the conversion has failed.
        """
        try:
            if is_depth:
                cv_tmp = self.cb.imgmsg_to_cv2(smi, encode)
                cv = np.array(cv_tmp, dtype=np.uint16)
            else:
                cv = self.cb.imgmsg_to_cv2(smi, encode)
            return cv
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def pose2map(self, map_info, x, y):
        """Convert from specified coordinates to 2D map coordinates.

        Args:
            map_info (nav_msgs/MapMetaData info): Information of map.
            x (float): Pose x to be converted.
            y (float): Pose y to be converted.

        Returns:
            float, float: x, y coordinates in map coordinate.
        """
        offset_x = int(map_info.origin.position.x / map_info.resolution) + int(map_info.width / 2.0)
        offset_y = int(map_info.origin.position.y / map_info.resolution) + int(map_info.height / 2.0)

        map_x = int(x / map_info.resolution) + int(map_info.width / 2.0) - offset_x
        map_y = int(y / map_info.resolution) + int(map_info.height / 2.0) - offset_y
        return map_x, map_y

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_hsr_gg_pkg/DetectDrivableAreaAction): Input informations.
        """
        if self.is_init == False:
            return

        # get new depth image
        current_ros_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if "subf_depth" not in self.update_ros_time.keys():
                continue
            if self.update_ros_time["subf_depth"] > current_ros_time:
                break

        self.lock.acquire()

        cv_d = self.smi2cv(self.smi_d, "passthrough", True)
        cv_blank = np.zeros((cv_d.shape[:2]), np.uint16) + np.iinfo(np.uint16).max
        cv_d_conv = cv2.bitwise_and(cv_d, cv_blank)

        # point cloud generation
        pcd = open3d.geometry.PointCloud.create_from_depth_image(
            open3d.geometry.Image(cv_d_conv), self.intrinsics)

        # downsampling
        pcd_down = pcd.voxel_down_sample(voxel_size=0.005)

        # cordinate transformation
        q = np.array([goal.transform_pose.orientation.x, 
                      goal.transform_pose.orientation.y, 
                      goal.transform_pose.orientation.z, 
                      goal.transform_pose.orientation.w])
        rot_mat = Rotation.from_quat(q)
        
        T = np.eye(4)
        T[:3, :3] = rot_mat.as_matrix()
        T[0, 3] = goal.transform_pose.position.x
        T[1, 3] = goal.transform_pose.position.y
        T[2, 3] = goal.transform_pose.position.z
        pcd_down_rot = copy.deepcopy(pcd_down).transform(T)

        # plane extraction and remove
        plane_model, inliers = pcd_down_rot.segment_plane(0.01, 3, 1000)
        pcd_not_plane = pcd_down_rot.select_by_index(inliers, invert=True)


        # apply to map
        pubt_map = OccupancyGrid()
        pubt_map.header = self.map.header
        pubt_map.info = self.map.info

        points = np.asarray([])
        data = []
        for i in range(len(self.map.data)):
            data.append(self.map.data[i])

        np_points = np.asarray(pcd_not_plane.points)
        points = []
        for x, y, z in np_points:
            points.append([x, y, z])
            map_x, map_y = self.pose2map(self.map.info, x, y)
            data[map_y * self.map.info.width + map_x] = 100
            
        pubt_map.data = tuple(data)
        self.pub_map.publish(pubt_map)

        # convert to point cloud
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = goal.frame
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        self.pub_obstacle_points.publish(pc2.create_cloud(header, fields, points))

        # change LocalCostmap
        self.p_robot_radius = rospy.get_param(rospy.get_name() + "/robot_radius", 0.25)
        self.client_global_costmap.update_configuration({
            "robot_radius": self.p_robot_radius})
        self.client_global_costmap_inflater.update_configuration({
            "inflation_radius": self.p_robot_radius})
        self.client_local_costmap.update_configuration({
            "robot_radius": self.p_robot_radius})
        self.client_local_costmap_inflater.update_configuration({
            "inflation_radius": self.p_robot_radius})
        
        result = DetectDrivableAreaResult(result = True)
        self.as_detect_drivable_area.set_succeeded(result)

        self.lock.release()
        self.prev_ros_time = current_ros_time

        # continue to publish until finished
        while not rospy.is_shutdown():
            if self.finish:
                self.finish = False
                self.client_global_costmap.update_configuration({
                    "robot_radius": 0.15})
                self.client_global_costmap_inflater.update_configuration({
                    "inflation_radius": 0.15})
                self.client_local_costmap.update_configuration({
                    "robot_radius": 0.15})
                self.client_local_costmap_inflater.update_configuration({
                    "inflation_radius": 0.15})
                return

            self.pub_obstacle_points.publish(pc2.create_cloud(header, fields, points))

        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = DetectDrivableAreaServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass 
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
