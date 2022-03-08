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

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *


class LibHSRSub:
    """HSR Subscribe library for WRS simlator."""
    
    def __init__(self, libutil, libopencv):
        """Constractor.

        Args:
            libutil (instance): Utility library instance
            libopencv (instance): OpenCV library instance
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {"util": libutil, "opencv": libopencv}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        self.joint_states = JointState()
        self.hand_image = Image()

        # ROS I/F
        self.sub_joint_states = rospy.Subscriber(
            "/hsrb/robot_state/joint_states",
            JointState,
            self.subf_joint_states,
            queue_size = 1)

        self.sub_hand_image = rospy.Subscriber(
            "/hsrb/hand_camera/image_raw",
            Image,
            self.subf_hand_image,
            queue_size = 1)

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def subf_joint_states(self, joint_states):
        """Callback function for joint states.

        Args:
            joint_states (sensor_msgs/JointState): Joint states.
        """
        self.lock.acquire()
        self.joint_states = joint_states
        self.lock.release()
    
        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return

    def subf_hand_image(self, hand_image):
        """Callback function for hand rgb image.

        Args:
            hand_image (sensor_msgs/Image): Hand rgb image.
        """
        self.lock.acquire()
        self.hand_image = hand_image
        self.lock.release()
    
        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return

    def get_joint_positions(self):
        """Get each joint position of the robot.

        Returns:
            dict[str, int]: Each joint position.

        Examples:
            >>> print(get_joint_positions())
            {"arm_lift_joint": 0.1, "arm_flex_joint": 0.17, ...}
        """
        current_ros_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.update_ros_time["subf_joint_states"] > current_ros_time:
                break

        self.lock.acquire()
        joint_potisions = {}
        for i in xrange(len(self.joint_states.name)):
            joint_potisions[self.joint_states.name[i]] = self.joint_states.position[i]
        self.lock.release()

        return joint_potisions

    def get_hand_image(self, is_cv=True):
        """Get hand camera images.

        Args:
            is_cv (bool, optional): Whether to convert to OpenCV format. Defaults to True.

        Returns:
            ndarray: If is_cv=True, return a OpenCV image.
                Returns False if the conversion fails.
            sensor_msgs/Image: If is_cv=False, return a sensor_msgs/Image.
        """
        current_ros_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.update_ros_time["subf_hand_image"] > current_ros_time:
                break
        self.lock.acquire()

        if is_cv:
            try:
                cv_rgb = self.lib["opencv"].smi2cv(self.hand_image, "bgr8")
                self.lock.release()
                return cv_rgb
            except:
                rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
                self.lock.release()
                return False
        else:
            smi_hand = self.hand_image
            self.lock.release()
            return smi_hand