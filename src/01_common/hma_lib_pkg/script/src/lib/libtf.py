#!/usr/bin/env python
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

import sys
import roslib
import tf

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

class LibTF:
    """Coordinate transformation library using TF."""
    
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.tfb = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()

        return
    
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def send_transform(self,
                       conv_frame,
                       origin_frame,
                       pose):
        """Coordinate transformation broadcast function.

        Args:
            conv_frame (str): Name of the converted frame.
            origin_frame (str): Name of the original frame.
            pose (geometry_msgs/Pose): Amount of offset from the coordinate system before transformation.
        """
        self.tfb.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            rospy.Time.now(),
            conv_frame,
            origin_frame)

        return

    def get_pose_without_offset(self,
                             conv_frame,
                             origin_frame):
        """Coordinate transformation without offset.

        Args:
            conv_frame (str): Name of the converted frame.
            origin_frame (str): Name of the original frame.

        Returns:
            geometry_msgs/Pose: Transformed pose.
        """
        while not rospy.is_shutdown():
            try:
                now = rospy.Time(0)
                self.tfl.waitForTransform(conv_frame, origin_frame, now, rospy.Duration(1.0))
                (trans, rot) = self.tfl.lookupTransform(conv_frame, origin_frame, now)
            except:
                rospy.logwarn("[" + rospy.get_name() + "]: Transform FAILURE")
                continue

            break

        return Pose(
            position = Point(
                x = trans[0],
                y = trans[1],
                z = trans[2]),
            orientation = Quaternion(
                x = rot[0],
                y = rot[1],
                z = rot[2],
                w = rot[3]))

    def get_pose_with_offset(self,
                             conv_frame,
                             origin_frame,
                             buf_frame,
                             pose):
        """Coordinate transformation with offset.

        Args:
            conv_frame (str): Name of the converted frame.
            origin_frame (str): Name of the original frame.
            buf_frame (str): Name of the buffer frame.
            pose (geometry_msgs/Pose): Amount of offset from the coordinate system before transformation.

        Returns:
            geometry_msgs/Pose: Transformed pose.
        """
        while not rospy.is_shutdown():
            self.tfb.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                rospy.Time.now(),
                buf_frame,
                origin_frame)

            try:
                now = rospy.Time(0)
                self.tfl.waitForTransform(conv_frame, buf_frame, now, rospy.Duration(1.0))
                (trans, rot) = self.tfl.lookupTransform(conv_frame, buf_frame, now)
            except:
                rospy.logwarn("[" + rospy.get_name() + "]: Transform FAILURE")
                continue

            break

        return Pose(
            position = Point(
                x = trans[0],
                y = trans[1],
                z = trans[2]),
            orientation = Quaternion(
                x = rot[0],
                y = rot[1],
                z = rot[2],
                w = rot[3]))

    def quaternion2euler(self, quaternion):
        """Convert from quaternion to euler angles.

        Args:
            quaternion (geometry_msgs/Quaternion): Quaternion before conversion.

        Returns:
            float: roll, pitch, yaw [rad].
        """
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))

        return roll, pitch, yaw

    def euler2quaternion(self, euler):
        """Convert from euler angles to quaternion.

        Args:
            euler (geometry_msgs/Pose2D, geometry_msgs/Point, list): Euler angles before conversion.

        Returns:
            geometry_msgs/Quaternion: Quaternion.
        """
        if isinstance(euler, Pose2D):
            q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.theta)
        elif isinstance(euler, Point):
            q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        elif isinstance(euler, list) or isinstance(euler, tuple):
            q = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])

        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def frame_exists(self, frame):
        """Check for the exists of frames.

        Args:
            frame (str): Name of the check frame.

        Returns:
            bool: Whether it exists or not.
        """
        return self.tfl.frameExists(frame)
