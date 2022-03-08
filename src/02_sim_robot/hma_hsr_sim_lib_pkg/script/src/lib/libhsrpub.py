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


class LibHSRPub:
    """HSR Publish library for WRS simlator."""
    
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.joint_states_topic = "/hsrb/robot_state/joint_states"

        # ROS I/F
        self.pub_command_velocity = rospy.Publisher(
            "/hsrb/command_velocity",
            Twist,
            queue_size = 1)

        self.pub_arm_trajectory_controller = rospy.Publisher(
            "/hsrb/arm_trajectory_controller/command",
            JointTrajectory,
            queue_size = 1)

        self.pub_head_trajectory_controller = rospy.Publisher(
            "/hsrb/head_trajectory_controller/command",
            JointTrajectory,
            queue_size = 1)

        return
    
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def omni_base_vel(self, x, y, yaw):
        """Move an omni_base at a specified speed.

        Args:
            x (float): Speed of x direction.
            y (float): Speed of y direction.
            yaw (float): Speed of yaw angle.
        """
        pubt_command_velocity = Twist()
        pubt_command_velocity.linear.x = x
        pubt_command_velocity.linear.y = y
        pubt_command_velocity.linear.z = 0.0
        pubt_command_velocity.angular.x = 0.0
        pubt_command_velocity.angular.y = 0.0
        pubt_command_velocity.angular.z = yaw
        
        self.pub_command_velocity.publish(pubt_command_velocity)
        return

    def whole_body_move_to_arm_positions(self, joints, time=1.0):
        """Transition a robot arm by specified values.

        Args:
            joints (dict[str, float]): Target value each arm joint.
                Note that joints that are not specified do not transition.
            time (float, optional): Time to transition completion.
                If the robot cannot reach the target in the specified time, it drives as fast as possible.
                Defaults to 1.0[sec].
        """
        joint_names = ["arm_lift_joint",
                       "arm_flex_joint",
                       "arm_roll_joint",
                       "wrist_flex_joint",
                       "wrist_roll_joint"]
        if joints.keys() != joint_names:
            joint_states = rospy.wait_for_message(self.joint_states_topic, JointState)
            currents = {}
            for i in xrange(len(joint_states.name)):
                currents[joint_states.name[i]] = joint_states.position[i]
            for name in joint_names:
                if name not in joints.keys():
                    joints[name] = currents[name]

        pubt_arm_trajectory_controller = JointTrajectory()
        pubt_arm_trajectory_controller.joint_names = joint_names
        pubt_arm_trajectory_controller.points = [
            JointTrajectoryPoint(
                positions = [joints["arm_lift_joint"],
                             joints["arm_flex_joint"], 
                             joints["arm_roll_joint"], 
                             joints["wrist_flex_joint"], 
                             joints["wrist_roll_joint"]],
                velocities = [0.0, 0.0, 0.0, 0.0, 0.0],
                time_from_start = rospy.Time(time))]

        self.pub_arm_trajectory_controller.publish(pubt_arm_trajectory_controller)
        return

    def whole_body_move_to_head_positions(self, joints, time=1.0):
        """Transition a robot head by specified values.

        Args:
            joints (dict[str, float]): Target value each arm joint.
                Note that joints that are not specified do not transition.
            time (float, optional): Time to transition completion. 
                If the robot cannot reach the target in the specified time, it drives as fast as possible.
                Defaults to 1.0[sec].
        """
        joint_names = ["head_pan_joint", "head_tilt_joint"]
        if joints.keys() != joint_names:
            joint_states = rospy.wait_for_message(self.joint_states_topic, JointState)
            currents = {}
            for i in xrange(len(joint_states.name)):
                currents[joint_states.name[i]] = joint_states.position[i]
            for name in joint_names:
                if name not in joints.keys():
                    joints[name] = currents[name]

        pubt_head_trajectory_controller = JointTrajectory()
        pubt_head_trajectory_controller.joint_names = joint_names
        pubt_head_trajectory_controller.points = [
            JointTrajectoryPoint(positions = [joints["head_pan_joint"], 
                                              joints["head_tilt_joint"]],
                                 velocities = [0.0, 0.0],
                                 time_from_start = rospy.Time(time))]

        self.pub_head_trajectory_controller.publish(pubt_head_trajectory_controller)
        return

    def whole_body_move_to_go(self, time=1.0):
        """Transition the whole body to go posture.

        Args:
            time (float, optional): Time to transition completion. 
                If the robot cannot reach the target in the specified time, it drives as fast as possible.
                Defaults to 1.0[sec].
        """
        self.whole_body_move_to_arm_positions({
            "arm_lift_joint": 0.0,
            "arm_flex_joint": 0.0,
            "arm_roll_joint": np.deg2rad(-90.0),
            "wrist_flex_joint":  np.deg2rad(-90.0),
            "wrist_roll_joint": 0.0},
            time)
        self.whole_body_move_to_head_positions({
            "head_pan_joint": 0.0,
            "head_tilt_joint": 0.0},
            time)
        return
