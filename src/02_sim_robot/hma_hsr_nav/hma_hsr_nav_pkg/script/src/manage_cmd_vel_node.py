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


GP_LOOP_RATE = 30.0


class ManageCmdVel:
    """Manage HSR's command velocity."""

    def __init__(self, run_enable=True):
        """Constructor.

        Args:
            run_enable (bool, optional): Run enable. Defaults to True.
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.run_enable = Bool(run_enable)

        # ROS I/F
        self.get_params()

        self.pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity",
                                           Twist,
                                           queue_size = 1)

        self.sub_run_enable = rospy.Subscriber(rospy.get_name() + "/run_enable",
                                               Bool,
                                               self.subf_run_enable,
                                               queue_size = 1)
        self.sub_cmd_vel = rospy.Subscriber("/hsrb/manage/command_velocity",
                                            Twist,
                                            self.subf_cmd_vel,
                                            queue_size = 1)

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def subf_run_enable(self, run_enable):
        """Callback function for run enable.

        Args:
            run_enable (std_msgs/Bool): Run enable.
        """
        self.run_enable = run_enable
        return

    def subf_cmd_vel(self, cmd_vel):
        """Callback function for command velocity.

        Args:
            cmd_vel (geometry_msgs/Twist): Command velocity.
        """
        try:
            self.main(cmd_vel)
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        return

    def get_params(self):
        """Get parameters."""
        self.p_enable = rospy.get_param(rospy.get_name() + "/enable", True)

        self.p_min_linear_vel = rospy.get_param(rospy.get_name() + "/linear_vel/min",
                                                GP_MIN_LINEAR_VEL)

        self.p_max_linear_vel = rospy.get_param(rospy.get_name() + "/linear_vel/max",
                                                GP_MAX_LINEAR_VEL)

        self.p_min_angular_vel = rospy.get_param(rospy.get_name() + "/angular_vel/min",
                                                 GP_MIN_ANGULAR_VEL)

        self.p_max_angular_vel = rospy.get_param(rospy.get_name() + "/angular_vel/max",
                                                 GP_MAX_ANGULAR_VEL)

    def main(self, cmd_vel):
        """Main function.

        Args:
            cmd_vel (geometry_msgs/Twist): Command velocity.
        """
        if self.run_enable.data == False:
            return

        current_ros_time = rospy.Time.now()
        self.lock.acquire()

        self.get_params()
        if self.p_enable:
            # manage minimum speed
            # linear x
            if abs(cmd_vel.linear.x) < self.p_min_linear_vel:
                if cmd_vel.linear.x >= 0:
                    cmd_vel.linear.x = self.p_min_linear_vel
                else:
                    cmd_vel.linear.x = -self.p_min_linear_vel

            # linear y
            if abs(cmd_vel.linear.y) < self.p_min_linear_vel:
                if cmd_vel.linear.y >= 0:
                    cmd_vel.linear.y = self.p_min_linear_vel
                else:
                    cmd_vel.linear.y = -self.p_min_linear_vel

            # angular
            if abs(cmd_vel.angular.z) < self.p_min_angular_vel:
                if cmd_vel.angular.z >= 0:
                    cmd_vel.angular.z = self.p_min_angular_vel
                else:
                    cmd_vel.angular.z = -self.p_min_angular_vel

            # manage maximum speed
            # linear x
            if abs(cmd_vel.linear.x) > self.p_max_linear_vel:
                if cmd_vel.linear.x >= 0:
                    cmd_vel.linear.x = self.p_max_linear_vel
                else:
                    cmd_vel.linear.x = -self.p_max_linear_vel

            # linear y
            if abs(cmd_vel.linear.y) > self.p_max_linear_vel:
                if cmd_vel.linear.y >= 0:
                    cmd_vel.linear.y = self.p_max_linear_vel
                else:
                    cmd_vel.linear.y = -self.p_max_linear_vel

            # angular
            if abs(cmd_vel.angular.z) > self.p_max_angular_vel:
                if cmd_vel.angular.z >= 0:
                    cmd_vel.angular.z = self.p_max_angular_vel
                else:
                    cmd_vel.angular.z = -self.p_max_angular_vel

            self.pub_cmd_vel.publish(cmd_vel)

        self.lock.release()
        self.prev_ros_time = current_ros_time
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = ManageCmdVel()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
