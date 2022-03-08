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

from curses import raw
import sys
import roslib

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libopencv import *
from libtf import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_sim_lib_pkg") + "/script/src/lib")
from libhsrutil import *
from libhsrpub import *
from libhsrmoveit import *

from hma_hsr_nav_action.msg import MotionSynthesisAction, MotionSynthesisResult


# Globals
GP_LOOP_RATE = 30.0


class MotionSynthesisServer:
    """Motion synthesis ROS Action server."""
    def __init__(self):
        self.update_ros_time = {}

        libutil = LibUtil()
        libopencv = LibOpenCV()
        libtf = LibTF()
        libhsrutil = LibHSRUtil(libutil, libtf)
        libhsrpub = LibHSRPub()
        libhsrmoveit = LibHSRMoveit(libtf, libtf, libhsrutil)

        self.lib = {"util": libutil,
                    "opencv": libopencv,
                    "tf": libtf,
                    "hsrutil": libhsrutil,
                    "hsrpub": libhsrpub,
                    "hsrmoveit": libhsrmoveit}

        # ROS I/F
        self.as_motion_synthesis = actionlib.SimpleActionServer("/motion_synthesis",
                                                                MotionSynthesisAction,
                                                                self.main)
        self.as_motion_synthesis.start()

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def execute(self, place, pose, tolerance, is_changed_arm_flex=False):
        """Execution function of the motion synthesis.

        Args:
            place (geometry_msgs/Pose2D): Goal place positions.
            pose (dict[str, float]): Transition posture.
            tolerance (geometry_msgs/Point): Timing of starting (distance from the goal).
            is_changed_arm_flex (bool, optional): Whether the arm flex joint is already in transition.
                Defaults to False.
        """
        while not rospy.is_shutdown():
            if self.as_motion_synthesis.is_preempt_requested():
                self.as_motion_synthesis.set_preempted()
                return

            if is_changed_arm_flex is False:
                if self.lib["hsrutil"].is_in_place(place, [tolerance.x, tolerance.y, np.deg2rad(180.0)]):
                    _pose = copy.deepcopy(pose)
                    _pose.arm_flex_joint = 0.0 if _pose.arm_flex_joint == 0.0 else -0.1
                    self.lib["hsrpub"].whole_body_move_to_arm_positions(
                        self.lib["hsrutil"].hsr_joints_to_dict(_pose), 0.1)
                    self.lib["hsrpub"].whole_body_move_to_head_positions(
                        self.lib["hsrutil"].hsr_joints_to_dict(pose), 0.1)

                    self.lib["hsrutil"].wait_goal_all_joints(
                        self.lib["hsrutil"].hsr_joints_to_dict(_pose),
                        pos_tolerance=0.01,
                        arm_ori_tolerance=np.deg2rad(10.0),
                        head_ori_tolerance=np.deg2rad(5.0),
                        arm_timeout=3.0,
                        head_timeout=3.0)

                    is_changed_arm_flex = True
            else:
                if self.lib["hsrutil"].is_in_place(place, [tolerance.x, tolerance.y, np.deg2rad(90.0)]):
                    self.lib["hsrpub"].whole_body_move_to_arm_positions(
                        self.lib["hsrutil"].hsr_joints_to_dict(pose), 0.1)
                    
                    self.lib["hsrutil"].wait_goal_all_joints(
                        self.lib["hsrutil"].hsr_joints_to_dict(pose),
                        pos_tolerance=0.01,
                        arm_ori_tolerance=np.deg2rad(10.0),
                        head_ori_tolerance=np.deg2rad(5.0),
                        arm_timeout=3.0,
                        head_timeout=3.0)

                    return 
        
    def execute_from_first(self, pose, joints):
        """Execution function of the motion synthesis (Only specified joints to be transitioned from the first).

        Args:
            pose (dict[str, float]): Transition posture.
            joints (list[str]): The specified joints are to be transitioned from the first.
        """
        go_posture = {"arm_lift_joint": 0.0,
                      "arm_flex_joint": 0.0,
                      "arm_roll_joint": np.deg2rad(-90.0),
                      "wrist_flex_joint":  np.deg2rad(-90.0),
                      "wrist_roll_joint": 0.0,
                      "head_pan_joint": 0.0,
                      "head_tilt_joint": 0.0}

        pose_dict = self.lib["hsrutil"].hsr_joints_to_dict(pose)
        exec_pose = {}
        for joint in joints:
            exec_pose[joint] = pose_dict[joint]
        
        for key in go_posture.keys():
            if key not in exec_pose.keys():
                exec_pose[key] = go_posture[key]

        self.lib["hsrpub"].whole_body_move_to_arm_positions(exec_pose, 0.1)
        self.lib["hsrpub"].whole_body_move_to_head_positions(exec_pose, 0.1)
        return

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_hsr_nav_action/MotionSynthesisAction): Input informations.
        """
        if goal.exec_from_first == [] and goal.apply_start_pose:
            self.lib["hsrpub"].whole_body_move_to_arm_positions(
                self.lib["hsrutil"].hsr_joints_to_dict(goal.start_pose), 0.1)
            self.lib["hsrpub"].whole_body_move_to_head_positions(
                self.lib["hsrutil"].hsr_joints_to_dict(goal.start_pose), 0.1)

        if goal.apply_goal_pose:
            is_changed_arm_flex = False
            if goal.exec_from_first != []:
                self.execute_from_first(goal.goal_pose, goal.exec_from_first)
                if "arm_flex_joint" in goal.exec_from_first and goal.goal_pose.arm_flex_joint < -0.1:
                    is_changed_arm_flex = True
            self.execute(goal.goal_place, goal.goal_pose, goal.goal_tolerance, is_changed_arm_flex)

        result = MotionSynthesisResult(result=True)
        self.as_motion_synthesis.set_succeeded(result)
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = MotionSynthesisServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
