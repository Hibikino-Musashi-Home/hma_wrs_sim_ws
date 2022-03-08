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

from hma_hsr_msgs.msg import HSRJoints


class LibHSRUtil:
    """HSR Utility library for WRS simlator."""
    
    def __init__(self, libutil, libtf):
        """Constractor.

        Args:
            libutil (instance): Utility library instance.
            libtf (instance): TF library instance.
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {"util": libutil, "tf": libtf}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        self.joint_states_topic = "/hsrb/robot_state/joint_states"

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return
    
    def dict_to_hsr_joints(self, joints):
        return HSRJoints(arm_lift_joint=joints["arm_lift_joint"],
                         arm_flex_joint=joints["arm_flex_joint"],
                         arm_roll_joint=joints["arm_roll_joint"],
                         wrist_flex_joint=joints["wrist_flex_joint"],
                         wrist_roll_joint=joints["wrist_roll_joint"],
                         head_pan_joint=joints["head_pan_joint"],
                         head_tilt_joint=joints["head_tilt_joint"])
                        
    def hsr_joints_to_dict(self, hsr_joints):
        return {"arm_lift_joint": hsr_joints.arm_lift_joint,
                "arm_flex_joint": hsr_joints.arm_flex_joint,
                "arm_roll_joint": hsr_joints.arm_roll_joint,
                "wrist_flex_joint": hsr_joints.wrist_flex_joint,
                "wrist_roll_joint": hsr_joints.wrist_roll_joint,
                "head_pan_joint": hsr_joints.head_pan_joint,
                "head_tilt_joint": hsr_joints.head_tilt_joint}

    def get_robot_position_on_map(self):
        """Get the current positions on map."""
        map2base = self.lib["tf"].get_pose_without_offset(
            self.robot_descriptor["FRAME_MAP"],
            self.robot_descriptor["FRAME_BASE_FOOTPRINT"])
        x = map2base.position.x
        y = map2base.position.y
        _, _, yaw = self.lib["tf"].quaternion2euler(map2base.orientation)
        return x, y, yaw
    
    def is_in_place(self, place, tolerance):
        """Determine if the robot is in a specified place.

        Args:
            place (list, geometry_msgs/Pose2D): Place positions.
            tolerance (list, geometry_msgs/Pose2D): Tolerances.

        Returns:
            bool: Whether or not the goal is reached.
        """
        if isinstance(place, list):
            return self.is_in_place(Pose2D(place[0], place[1], place[2]), tolerance)
        if isinstance(tolerance, list):
            return self.is_in_place(place, Pose2D(tolerance[0], tolerance[1], tolerance[2]))

        x, y, yaw = self.get_robot_position_on_map()
        if abs(x - place.x) <= tolerance.x:
            if abs(y - place.y) <= tolerance.y:
                # if abs(yaw - place.theta) <= tolerance.theta:
                return True
        return False

    def _wait_goal(self, joints, config, timeout=5.0):
        """Waiting until joints reach the target value.

        Args:
            joints (dict[str, float]): Target value each arm joint.
            config (dict[str, float]): Configs specifying each joint and its tolerance.
            timeout (float, optional): Timeout time. Defaults to 5.0[sec].

        Returns:
            bool: Whether the target joint value was reached or not.
        """
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if rospy.Time.now() - start > rospy.Duration(timeout): 
                return False

            joint_states = rospy.wait_for_message(self.joint_states_topic, JointState)
            currents = {}
            for i in xrange(len(joint_states.name)):
                currents[joint_states.name[i]] = joint_states.position[i]
            goal_joints = []
            for key in config.keys():
                if key in joints.keys():
                    if abs(currents[key] - joints[key]) < config[key]:
                        goal_joints.append(key)
            
            if len(goal_joints) == len(config):
                return True

    def wait_goal_arm_joints(self, joints,
                             pos_tolerance=0.01,
                             ori_tolerance=0.017,
                             timeout=5.0):
        """Waiting until arm joints reach the target value.

        Args:
            joints (dict[str, float]): Target value each arm joint.
            pos_tolerance (float, optional): The tolerance of positions. Defaults to 0.01[m].
            ori_tolerance (float, optional): The tolerance of orientations. Defaults to 0.017[rad].
            timeout (float, optional): Timeout time. Defaults to 5.0[sec].

        Returns:
            bool: Whether the target joint value was reached or not.
        """
        config = {"arm_lift_joint": pos_tolerance,
                   "arm_flex_joint": ori_tolerance,
                   "arm_roll_joint": ori_tolerance,
                   "wrist_flex_joint": ori_tolerance,
                   "wrist_roll_joint": ori_tolerance}
        return self._wait_goal(joints, config, timeout)

    def wait_goal_head_joints(self, joints,
                              ori_tolerance=0.017,
                              timeout=5.0):
        """Waiting until head joints reach the target value.

        Args:
            joints (dict[str, float]): Target value each head joint.
            ori_tolerance (float, optional): The tolerance of orientations. Defaults to 0.017[rad].
            timeout (float, optional): Timeout time. Defaults to 5.0[sec].

        Returns:
            bool: Whether the target joint value was reached or not.
        """
        config = {"head_pan_joint": ori_tolerance,
                   "head_tilt_joint": ori_tolerance}
        return self._wait_goal(joints, config, timeout)

    def wait_goal_all_joints(self, joints,
                             pos_tolerance=0.01,
                             arm_ori_tolerance=0.017,
                             head_ori_tolerance=0.017,
                             arm_timeout=5.0,
                             head_timeout=5.0):
        """Waiting until arm and head joints reach the target value.

        Args:
            joints (dict[str, float]): Target value each arm joint.
            pos_tolerance (float, optional): The tolerance of positions. Defaults to 0.01[m].
            arm_ori_tolerance (float, optional): The tolerance of arm orientations. Defaults to 0.017[rad].
            head_ori_tolerance (float, optional): The tolerance of head orientations. Defaults to 0.017[rad].
            arm_timeout (float, optional): Timeout time for arm. Defaults to 5.0[sec].
            head_timeout (float, optional): Timeout time for head. Defaults to 5.0[sec].

        Returns:
            tuple[bool]: Whether the target joint value was reached or not.
        """
        arm = self.wait_goal_arm_joints(joints, pos_tolerance, arm_ori_tolerance, arm_timeout)
        head = self.wait_goal_head_joints(joints, head_ori_tolerance, head_timeout)
        return all((arm, head))