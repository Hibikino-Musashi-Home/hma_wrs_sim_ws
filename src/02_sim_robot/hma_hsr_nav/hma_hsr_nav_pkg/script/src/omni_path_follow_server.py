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

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libtf import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_sim_lib_pkg") + "/script/src/lib")
from libhsrutil import *


# Global
GP_LOOP_RATE = 30.0


class OmniPathFollowServer:
    """Omni path follower ROS Action server."""

    def __init__(self):
        self.update_ros_time = {}

        libutil = LibUtil()
        libtf = LibTF()
        libhsrutil = LibHSRUtil(libutil, libtf)
        self.lib = {"util": libutil, "tf": libtf, "hsrutil": libhsrutil}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        self.cmd_vel = Twist()
        self.goal_id = 0

        # ROS I/F
        self.pub_goal_path_follower = rospy.Publisher("/path_follow_action/goal",
                                                      PathFollowerActionGoal,
                                                      queue_size = 1)
        self.pub_cancel_path_follower = rospy.Publisher("/path_follow_action/cancel",
                                                        GoalID,
                                                        queue_size = 1)
        self.pub_status = rospy.Publisher(rospy.get_name() + "/goal_status",
                                          GoalStatus,
                                          queue_size = 1)

        self.as_omni_path_follower = actionlib.SimpleActionServer(
            "/omni_path_follow",
            OmniPathFollowerAction,
            self.main)
        self.as_omni_path_follower.start()

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def pub_goal_status(self, goal_id, status):
        """Publish the status of action.

        Args:
            goal_id (int): Goal ID.
            status (int): Status.
        """
        pub = GoalStatus(goal_id=GoalID(id=str(goal_id)),
                         status=status)
        self.pub_status.publish(pub)
        return

    def return_result(self, goal_id, result):
        """Return result.

        Args:
            goal_id (str): Goal ID.
            result (hma_hsr_nav_action/OmniPathFollowerResult): Result of action.
        """
        goal_id = GoalID()
        goal_id.stamp = rospy.Time.now()
        goal_id.id = str(goal_id)
        self.pub_cancel_path_follower.publish(goal_id)
        rospy.sleep(0.1)

        self.as_omni_path_follower.set_succeeded(
            OmniPathFollowerResult(result=result))

        # self.pub_goal_status(GoalStatus.ABORTED, goal_id+1)
        if result:
            self.pub_goal_status(goal_id, GoalStatus.SUCCEEDED)
        else:
            self.pub_goal_status(goal_id, GoalStatus.ABORTED)
        return

    def start(self, goal_id, pose, path):
        """Start of path follow.

        Args:
            goal_id (str): Goal ID.
            pose (sensor_msgs/Pose2D): Goal pose.
            path (geometry_msgs/PoseStamped[]): Path to follow.
        """
        goal = PathFollowerActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.robot_descriptor["FRAME_MAP"]
        goal.goal_id.stamp = rospy.Time.now()
        goal.goal_id.id = str(goal_id)
        goal.goal.path.poses = path

        self.pub_goal_path_follower.publish(goal)

        pre_pos = None
        pre_time_stamp = 0
        pre_pos_abo = None
        pre_time_stamp_abo = 0
        while not rospy.is_shutdown():
            if self.as_omni_path_follower.is_preempt_requested():
                self.as_omni_path_follower.set_preempted()
                return

            self.pub_goal_status(goal_id, GoalStatus.ACTIVE)

            # Failure: Stop for 3 seconds except near the goal.
            if not self.lib["hsrutil"].is_in_place(pose, [0.05, 0.05, np.deg2rad(10.0)]):
                pre_pos = None
                pre_time_stamp = 0
                if pre_pos_abo is not None:
                    cur_pos_abo = self.lib["hsrutil"].get_robot_position_on_map()
                    cur_time_stamp_abo = rospy.Time.now()

                    if cur_time_stamp_abo - pre_time_stamp_abo > rospy.Duration(secs=3.0):
                        self.return_result(goal_id, False)
                        return

                    # is moving
                    if abs(pre_pos_abo[0] - cur_pos_abo[0]) > 0.01:
                        if abs(pre_pos_abo[1] - cur_pos_abo[1]) > 0.01:
                            if abs(pre_pos_abo[2] - cur_pos_abo[2]) > np.deg2rad(1.0):
                                pre_pos_abo = cur_pos_abo
                                pre_time_stamp_abo = cur_time_stamp_abo
                                
                else:
                    pre_pos_abo = self.lib["hsrutil"].get_robot_position_on_map()
                    pre_time_stamp_abo = rospy.Time.now()

            # Success: Stop for 1 second near the goal.
            else:
                pre_pos_abo = None
                pre_time_stamp_abo = 0
                if pre_pos is not None:
                    cur_pos = self.lib["hsrutil"].get_robot_position_on_map()
                    cur_time_stamp = rospy.Time.now()

                    if cur_time_stamp - pre_time_stamp > rospy.Duration(secs=1.0):
                        self.return_result(goal_id, True)
                        return

                    # is moving
                    if abs(pre_pos[0] - cur_pos[0]) > 0.01:
                        if abs(pre_pos[1] - cur_pos[1]) > 0.01:
                            if abs(pre_pos[2] - cur_pos[2]) > np.deg2rad(1.0):
                                pre_pos = cur_pos
                                pre_time_stamp = cur_time_stamp
                                
                else:
                    pre_pos = self.lib["hsrutil"].get_robot_position_on_map()
                    pre_time_stamp = rospy.Time.now()

    def cancel(self, goal_id):
        """Cancel goal.

        Args:
            goal_id (str): Goal ID to cancel.
        """
        pub_goal_id = GoalID()
        pub_goal_id.stamp = rospy.Time.now()
        pub_goal_id.id = str(goal_id)
        self.pub_cancel_path_follower.publish(pub_goal_id)
        rospy.sleep(0.1)
        
        result = OmniPathFollowerResult(result=True)
        self.as_omni_path_follower.set_succeeded(result)

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_hsr_nav_action/OmniPathFollowerAction): Input informations.
        """
        if goal.action == PathFollow.START:
            self.start(self.goal_id, goal.pose, goal.path)
        elif goal.action == PathFollow.CANCEL:
            self.cancel(self.goal_id)

        self.goal_id += 1
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = OmniPathFollowServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
