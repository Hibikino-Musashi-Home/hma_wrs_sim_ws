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

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libtf import *


# Globals
GP_LOOP_RATE = 30.0

class PublishGlobalPose:
    """Publish global pose of HSR."""

    def __init__(self, run_enable=True):
        """Constructor.

        Args:
            run_enable (bool, optional): Run enable. Defaults to True.
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        libutil = LibUtil()
        libtf = LibTF()
        self.lib = {"util": libutil, "tf": libtf}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        self.run_enable = Bool(run_enable)

        # ROS I/F
        self.pub_global_pose = rospy.Publisher("/global_pose",
                                               PoseStamped,
                                               queue_size=1)

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def main(self):
        """Main function."""
        if self.run_enable.data == False:
            return

        current_ros_time = rospy.Time.now()
        self.lock.acquire()

        try:
            global_pose = PoseStamped()
            global_pose.header.stamp = rospy.Time.now()
            global_pose.header.frame_id = self.robot_descriptor["FRAME_MAP"]
            global_pose.pose = self.lib["tf"].get_pose_without_offset(
                self.robot_descriptor["FRAME_MAP"],
                self.robot_descriptor["FRAME_BASE_FOOTPRINT"])
            self.pub_global_pose.publish(global_pose)
        except:
            pass

        self.lock.release()
        self.prev_ros_time = current_ros_time
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = PublishGlobalPose()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.main()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
