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

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_nav_pkg") + "/script/import")
from hsr_nav_import import *


class LibHSRNav:
    """HSR Navigation library."""
    def __init__(self, libutil, libtf, libhsrutil):
        """Constractor.

        Args:
            libutil (instance): Utility library instance.
            libtf (instance): TF library instance.
            libhsrutil (instance): HSR Utility library instance.
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {"util": libutil, "tf": libtf, "hsrutil": libhsrutil}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        # ROS I/F
        self.pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity",
                                           Twist,
                                           queue_size=1)

        self.srv_make_plan = rospy.ServiceProxy(
            "/hma_move_base/GlobalPlanner/make_plan", GetPlan)

        self.ac_path_follow = actionlib.SimpleActionClient(
            "/omni_path_follow", OmniPathFollowerAction)
        self.ac_path_follow.wait_for_server()

        self.ac_motion_synthesis = actionlib.SimpleActionClient(
            "/motion_synthesis", MotionSynthesisAction)
        self.ac_motion_synthesis.wait_for_server()

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def set_change_vel_enable(self, enable):
        """Set with/without velocity change.

        Args:
            enable (bool): enable or disable
        """
        rospy.set_param("/manage_cmd_vel_node/enable", enable)
        return

    def set_min_max_vel(self,
                        min_linear_vel=GP_MIN_LINEAR_VEL,
                        max_linear_vel=GP_MAX_LINEAR_VEL,
                        min_angular_vel=GP_MIN_ANGULAR_VEL,
                        max_angular_vel=GP_MAX_ANGULAR_VEL):
        """Change minimum and maximum velocities.

        Args:
            min_linear_vel (float, optional): minimum linear velocity. Defaults to GP_MIN_LINEAR_VEL.
            max_linear_vel (float, optional): maximum linear velocity. Defaults to GP_MAX_LINEAR_VEL.
            min_angular_vel (float, optional): minimum angular velocity. Defaults to GP_MIN_ANGULAR_VEL.
            max_angular_vel (float, optional): maximum angular velocity. Defaults to GP_MAX_ANGULAR_VEL.
        """
        self.set_change_vel_enable(True)
        rospy.set_param("/manage_cmd_vel_node/linear_vel/min", min_linear_vel)
        rospy.set_param("/manage_cmd_vel_node/linear_vel/max", max_linear_vel)
        rospy.set_param("/manage_cmd_vel_node/angular_vel/min", min_angular_vel)
        rospy.set_param("/manage_cmd_vel_node/angular_vel/max", max_angular_vel)
        return

    def cancel_path_follow_goals(self):
        """Cancel Path Follow action."""
        self.ac_path_follow.cancel_all_goals()
        goal = OmniPathFollowerGoal(action=PathFollow.CANCEL)
        self.ac_path_follow.send_goal(goal)
        self.ac_path_follow.wait_for_result()
        return

    def _navigation(self, pose, frame,
                    goal_tolerance=[],
                    vel_minmax=[],
                    is_wait_motion_synthesis=True):
        if vel_minmax == []:
            self.set_min_max_vel()
        else:
            self.set_min_max_vel(*vel_minmax)

        state = NavState.START
        send_ros_time = None
        while not rospy.is_shutdown():
            # goal decision
            if goal_tolerance != []:
                if self.lib["hsrutil"].is_in_place(pose, goal_tolerance):
                    self.cancel_path_follow_goals()
                    state = NavState.SUCCESS

            # start
            if state == NavState.START:
                rospy.loginfo("[" + rospy.get_name() + "]: NavState.START")

                # TF
                goal_place = PoseStamped()
                goal_place.header.stamp = rospy.Time.now()
                goal_place.header.frame_id = frame
                goal_place.pose.position = Point(pose.x, pose.y, 0.0)
                goal_place.pose.orientation = self.lib["tf"].euler2quaternion((0.0, 0.0, pose.theta))
                self.lib["tf"].send_transform("goal_place", frame, goal_place.pose)

                # make path
                x, y, yaw = self.lib["hsrutil"].get_robot_position_on_map()
                make_path = PoseStamped()
                make_path.header.frame_id = frame
                make_path.pose.position = Point(x, y, 0.0)
                make_path.pose.orientation = self.lib["tf"].euler2quaternion((0.0, 0.0, yaw))
                path = self.srv_make_plan(make_path, goal_place, 0.01).plan.poses

                if len(path) == 0:
                    self.ac_motion_synthesis.cancel_all_goals()
                    return False

                # execute
                goal = OmniPathFollowerGoal()
                goal.action = PathFollow.START
                goal.pose = pose
                goal.path = path
                self.ac_path_follow.send_goal(goal)
                send_ros_time = rospy.Time.now()

                state = NavState.ACTIVE

            # active
            elif state == NavState.ACTIVE:
                # rospy.loginfo("[" + rospy.get_name() + "]: NavState.ACTIVE")

                self.ac_path_follow.wait_for_result()
                if self.ac_path_follow.get_result().result:
                    state = NavState.SUCCESS
                else:
                    state = NavState.FAILURE

            # success
            elif state == NavState.SUCCESS:
                if send_ros_time is None:
                    return True
                if is_wait_motion_synthesis:
                    self.ac_motion_synthesis.wait_for_result()

                rospy.loginfo("[" + rospy.get_name() + "]: NavState.SUCCESS")
                return True

            # failure
            elif state == NavState.FAILURE:
                rospy.loginfo("[" + rospy.get_name() + "]: NavState.FAILURE")

                # FIXME: Temporary handling of stuck bug.
                sec = 0.5
                for i in xrange(int(sec / 0.1)):
                    cmd_vel = Twist()
                    cmd_vel.linear.x = -0.2
                    self.pub_cmd_vel.publish(cmd_vel)
                    rospy.sleep(0.1)

                self._navigation(pose, frame, goal_tolerance, vel_minmax, is_wait_motion_synthesis)
                return True

    def go_abs(self, pose, 
               frame="map",
               vel_minmax=[],
               motion_synthesis_config={}):
        """Move in absolute coordinates. It loops infinitely until the goal is reached.

        Args:
            pose (geometry_msgs/Pose2D, list[geometry_msgs/Pose2D]): Goal place positions.
                Multiple positions can also be specified in a list. 
                In such a case, the robot is moved in order.
            frame (str): The name of reference frame.
            vel_minmax (list): Minimum and maximum velocities.
            motion_synthesis_config (dict, optional): If you want to apply the motion synthesis, set the configuration.
                Defaults to {} (Disable).
                The keys for config are as follows,
                        If "auto" is specified, it is automatically set to the go posture of HSR.
                    goal_pose: Transition posture at goal.
                    exec_from_first: Joint name list to transition from the first.
                        If "all" is specified, all specified joints execute from first.
                    goal_tolerance: Timing of starting motion synthesis (distance from the goal).
                    is_wait: Whether to wait for the completion of motion synthesis.
        Examples:
            base_go_abs(Pose2D(0.0, 1.0, 1.57), 
                        motion_synthesis_config={"start_pose":"auto", 
                                                 "goal_pose":{"arm_lift_joint": 0.1, ...}, 
                                                 "exec_from_first": ["arm_lift_joint"]},
                                                 "goal_tolerance": Point(0.3, 0.3, 0.0),
                                                 "is_wait": True)
        """
        def exec_motion_synthesis(pose, config):
            goal = MotionSynthesisGoal()
            goal.goal_place = pose
            goal.apply_start_pose = False
            if "start_pose" in config.keys():
                goal.apply_start_pose = True
                if config["start_pose"] == "auto":
                    start_pose = {"arm_lift_joint": 0.0,
                                  "arm_flex_joint": 0.0,
                                  "arm_roll_joint": np.deg2rad(-90.0),
                                  "wrist_flex_joint": np.deg2rad(-90.0),
                                  "wrist_roll_joint": 0.0,
                                  "head_pan_joint": 0.0,
                                  "head_tilt_joint": np.deg2rad(-30.0)}
                    goal.start_pose = self.lib["hsrutil"].dict_to_hsr_joints(start_pose)
                else:
                    goal.start_pose = self.lib["hsrutil"].dict_to_hsr_joints(config["start_pose"])
            goal.apply_goal_pose = False
            if "goal_pose" in config.keys():
                goal.apply_goal_pose = True
                goal.goal_pose =  self.lib["hsrutil"].dict_to_hsr_joints(config["goal_pose"])
                if "exec_from_first" in config.keys():
                    if config["exec_from_first"] == "all":
                        goal.exec_from_first = config["goal_pose"].keys()
                    else:
                        goal.exec_from_first = config["exec_from_first"]
                if "goal_tolerance" in config.keys():
                    goal.goal_tolerance = config["goal_tolerance"]
                else:
                    goal.goal_tolerance = Point(0.3, 0.3, 0.0)
            self.ac_motion_synthesis.send_goal(goal)

        if "is_wait" not in motion_synthesis_config.keys():
            motion_synthesis_config["is_wait"] = True

        if isinstance(pose, list):
            exec_motion_synthesis(pose[-1], motion_synthesis_config)
            for i in range(len(pose) - 1):
                stats = self._navigation(pose[i], frame, [0.1, 0.1, np.deg2rad(180.0)], vel_minmax, motion_synthesis_config["is_wait"])
                if stats is False:
                    return False
            else:
                return self._navigation(pose[-1], frame, [], vel_minmax, motion_synthesis_config["is_wait"])
        else:
            exec_motion_synthesis(pose, motion_synthesis_config)
            return self._navigation(pose, frame, [], vel_minmax, motion_synthesis_config["is_wait"])