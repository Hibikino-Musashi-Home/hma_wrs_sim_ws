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
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import numpy as np
import rospy
import actionlib
import copy
import statistics
import open3d

from enum import IntEnum
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from visualization_msgs.msg import Marker

from hma_common_action.msg import GraspPoseEstimationAction
from hma_common_action.msg import GraspPoseEstimationResult

sys.path.remove("/opt/ros/melodic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge

# Global
GP_LOOP_RATE = 30.0

class GraspPose(IntEnum):
    TOP = 0
    FRONT = 1
    SIDE = 2
    SUCTION = 3


class GraspPoseEstimationServer:
    """Grasp pose estimation ROS Action server."""

    def __init__(self):
        self.cb = CvBridge()

        # ROS I/F
        self.p_camera_info = rospy.get_param(rospy.get_name() + "/camera_info",
                                             "/camera/depth_registered/camera_info")
        self.p_size_th = {"min": {"height": 0.0},
                          "max": {"width": 0.0}}
        self.p_size_th["min"]["height"] = rospy.get_param(rospy.get_name() + "/size_th/min/height", 0.15)
        self.p_size_th["max"]["width"] = rospy.get_param(rospy.get_name() + "/size_th/max/width", 0.14)

        self.pub_dbg = rospy.Publisher(rospy.get_name() + "/dbg",
                                       PointCloud2,
                                       queue_size = 1)
        self.pub_marker = rospy.Publisher(rospy.get_name() + "/marker/3d_bounding_box",
                                          Marker,
                                          queue_size = 1)

        self.camera_info = rospy.wait_for_message(self.p_camera_info, CameraInfo)
        self.intrinsics = open3d.camera.PinholeCameraIntrinsic()
        self.intrinsics.set_intrinsics(width=self.camera_info.width,
                                       height=self.camera_info.height,
                                       fx=self.camera_info.K[0],
                                       fy=self.camera_info.K[4],
                                       cx=self.camera_info.K[2],
                                       cy=self.camera_info.K[5])
                                       
        self.as_grasp_pose_estimation = actionlib.SimpleActionServer(
            "/grasp_pose_estimation",
            GraspPoseEstimationAction,
            self.main)
        self.as_grasp_pose_estimation.start()

        return

    def delete(self):
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

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_common_action/GraspPoseEstimationAction): Input informations.
        """
        # object mask
        cv_mask = self.smi2cv(goal.mask, "mono8")
        cv_mask_obj = np.zeros((self.camera_info.height, self.camera_info.width), np.uint16)
        # cv_mask_obj[:, :] = np.where(cv_mask == goal.index+1, np.iinfo(np.uint16).max, 0)
        cv_mask_obj[:, :] = np.where(cv_mask != 0, np.iinfo(np.uint16).max, 0)

        # point cloud ganeration
        cv_d = self.smi2cv(goal.d, "passthrough", True)
        cv_d_obj = cv2.bitwise_and(cv_d, cv_mask_obj)
        kernel = np.ones((20, 20), np.uint16)
        cv_d_obj = cv2.morphologyEx(cv_d_obj, cv2.MORPH_CLOSE, kernel)
        pcd = open3d.geometry.PointCloud.create_from_depth_image(
                open3d.geometry.Image(cv_d_obj),
                self.intrinsics)

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
        pcd_rot = copy.deepcopy(pcd).transform(T)

        # clustering
        labels = np.array(pcd_rot.cluster_dbscan(eps=0.01, min_points=10, print_progress=False))
        if labels.max() < 0:
            rospy.logwarn("[" + rospy.get_name() + "]: No cluster detected.")
            result = GraspPoseEstimationResult(is_valid_grasp_pose = False)
            self.as_grasp_pose_estimation.set_succeeded(result)
            return

        # cluster extraction
        indexs = np.where(labels == statistics.mode(labels))[0]
        pcd_obj = pcd_rot.select_by_index(indexs)

        # object size estimation
        obj_min = pcd_obj.get_min_bound()
        obj_max = pcd_obj.get_max_bound()
        obj_w = min(abs(obj_max[0] - obj_min[0]), abs(obj_max[1] - obj_min[1]))
        obj_d = max(abs(obj_max[0] - obj_min[0]), abs(obj_max[1] - obj_min[1]))
        obj_h = abs(obj_max[2] - obj_min[2])

        # grasp pose estimation
        direction = GraspPose.TOP
        angle = 0.0
        if goal.specific_direction != -1:
            direction = goal.specific_direction
        else:
            if obj_h < self.p_size_th["min"]["height"]:
                rospy.loginfo("[" + rospy.get_name() + "]: Grasp TOP-0")
                direction = GraspPose.TOP
            else:
                if obj_w > p_size_th["max"]["width"]:
                    rospy.loginfo("[" + rospy.get_name() + "]: Grasp TOP-1")
                    direction = GraspPose.TOP
                else:
                    rospy.loginfo("[" + rospy.get_name() + "]: Grasp FRONT-0")
                    direction = GraspPose.FRONT

        # PCA (only grasp from top)
        if direction == GraspPose.TOP:
            def unit_vector(vector):
                return vector / np.linalg.norm(vector)
            def angle_between(v1, v2):
                v1_u = unit_vector(v1)
                v2_u = unit_vector(v2)
                return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

            np_points = np.asarray(pcd_obj.points)
            pca = PCA(n_components=1)
            pca.fit(np_points[:, :2])
            angle = angle_between((1, 0, 0), (pca.components_[0, 0], pca.components_[0, 1], 0))

        # grasp pose
        euler = np.array([0, 0, angle])
        rot_mat = Rotation.from_euler("XYZ", euler)
        obj_c = pcd_obj.get_center()
        obj_pose = Pose(Point(obj_c[0], obj_c[1], obj_c[2]),
                        Quaternion(rot_mat.as_quat()[0],
                                   rot_mat.as_quat()[1],
                                   rot_mat.as_quat()[2],
                                   rot_mat.as_quat()[3]))

        # marker for debug
        marker = Marker()

        # delete
        marker.action = Marker.DELETEALL
        self.pub_marker.publish(marker)

        # add
        marker.header.frame_id = goal.frame
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration.from_sec(30.0)
        marker.ns = "grasp_obj"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.color.r = 1.0 if direction == GraspPose.FRONT else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if direction == GraspPose.FRONT else 1.0
        marker.color.a = 0.5
        marker.pose = obj_pose
        marker.scale.x = obj_d
        marker.scale.y = obj_w
        marker.scale.z = obj_h
        self.pub_marker.publish(marker)

        # return result
        result = GraspPoseEstimationResult()
        result.is_valid_grasp_pose = True
        result.id = goal.id
        result.pose = obj_pose
        result.d = obj_d
        result.w = obj_w
        result.h = obj_h
        result.direction = direction
        self.as_grasp_pose_estimation.set_succeeded(result)

        # open3d.visualization.draw_geometries([pcd_obj]) # for debug
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = GraspPoseEstimationServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass 
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
