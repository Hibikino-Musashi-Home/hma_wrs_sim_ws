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

import os
import sys
import rospy
import threading

from std_msgs.msg import Bool


# Global
GP_LOOP_RATE = 10.0


class ManageTaskTime:
    """Manage task time."""
    
    def __init__(self, run_enable=True):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.run_enable = Bool(run_enable)

        # ROS I/F
        self.p_time = rospy.get_param(rospy.get_name() + "/time", 10.0)
    
        self.sub_run_enable = rospy.Subscriber(rospy.get_name() + "/run_enable",
                                        Bool,
                                        self.subf_run_enable,
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
        if self.run_enable.data is True:
            rospy.loginfo("[" + rospy.get_name() + "]: Start.")
            rospy.loginfo("[" + rospy.get_name() + "]: TIME: " + str(self.p_time) + " minutes.")
        return


    def main(self):
        if self.run_enable.data == False:
            return

        rospy.sleep(self.p_time * 60.0)
        rospy.loginfo("[" + rospy.get_name() + "]: Finish.")
        sys.exit(1)
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = ManageTaskTime(False)
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.main()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()