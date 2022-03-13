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
import numpy as np

from subprocess import Popen, call

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Pose
from gazebo_msgs.msg import ModelState, ModelStates

from gazebo_msgs.srv import SetModelState


# Globals
GP_LOOP_RATE = 30.0

class PrepareTask:
    """Prepare task for the WRS Simulator."""

    def __init__(self, run_enable=True):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.run_enable = Bool(run_enable)
        self.gazebo_model_states = ModelStates()

        # ROS I/F
        self.p_prep = rospy.get_param(rospy.get_name() + "/prep", True)
        self.p_seed = str(rospy.get_param(rospy.get_name() + "/seed", -1))
        self.p_percategory = str(rospy.get_param(rospy.get_name() + "/percategory", 5))
        self.p_obstacles = str(rospy.get_param(rospy.get_name() + "/obstacles", 3))
        self.p_perrow = str(rospy.get_param(rospy.get_name() + "/perrow", 4))
        self.p_drawer = rospy.get_param(rospy.get_name() + "/drawer_open", False)
        self.p_delete_prefix = rospy.get_param(rospy.get_name() + "/delete_prefix", "task")

        self.sub_gazebo_model_states = rospy.Subscriber("/gazebo/model_states",
                                                        ModelStates,
                                                        self.subf_gazebo_model_states,
                                                        queue_size=1)

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

    def subf_gazebo_model_states(self, gazebo_model_states):
        """Callback function for gazebo model states.

        Args:
            gazebo_model_states (gazebo_msgs/ModelStates): Gazebo model states.
        """
        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        try:
            self.main(gazebo_model_states)
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")

        return

    def delete_models(self, gazebo_model_states, prefix):
        """Delete gazebo models.

        Args:
            gazebo_model_states (gazebo_msgs/ModelStates): Gazebo model states.
            prefix (str): Model name prefix to be deleted.
        """
        for name in gazebo_model_states.name:
            if prefix in name:
                cmd = ['rosservice', 'call', 'gazebo/delete_model', '{model_name: ' + str(name) + '}']
                call(cmd)
                rospy.loginfo("[" + rospy.get_name() + "]: Delete " + str(name))
        return

    def close_drawers(self):
        """Close three drawers."""
        rospy.wait_for_service("/gazebo/set_model_state")
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.wait_for_service("/gazebo/set_model_state")

        point = [Point(-2.7, 0.67, 0.095),
                 Point(-2.7, 1.0, 0.095),
                 Point(-2.7, 1.0, 0.35)]
        for i, name in enumerate(["trofast_1", "trofast_2", "trofast_3"]):
            model_state = ModelState()
            model_state.model_name = name
            model_state.pose = Pose(point[i], Quaternion(0, 0, 0, 1))
            set_model_state(model_state)

        return

    def open_drawers(self):
        """Open three drawers."""
        rospy.wait_for_service("/gazebo/set_model_state")
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.wait_for_service("/gazebo/set_model_state")

        point = [Point(-2.7 + 0.3, 0.67, 0.095),
                 Point(-2.7 + 0.3, 1.0, 0.095),
                 Point(-2.7 + 0.3, 1.0, 0.35)]
        for i, name in enumerate(["trofast_1", "trofast_2", "trofast_3"]):
            model_state = ModelState()
            model_state.model_name = name
            model_state.pose = Pose(point[i], Quaternion(0, 0, 0, 1))
            set_model_state(model_state)

    def main(self, gazebo_model_states):
        """Main function.

        Args:
            gazebo_model_states (gazebo_msgs/ModelStates): Gazebo model states.
        """
        if self.run_enable.data == False:
            return
        
        current_ros_time = rospy.Time.now()
        self.lock.acquire()

        # task preparation
        if self.p_prep:
            self.close_drawers()
            self.delete_models(gazebo_model_states, self.p_delete_prefix)

            if int(self.p_seed) < 0:
                self.p_seed = str(np.random.randint(0, 10000))
                rospy.loginfo("[" + rospy.get_name() + "]: Set random seed " + str(self.p_seed))
            cmd = "rosrun tmc_wrs_gazebo_worlds spawn_objects" \
                + " --seed " + str(self.p_seed) \
                + " --percategory "+ str(self.p_percategory) \
                + " --obstacles " + str(self.p_obstacles) \
                + " --perrow " + str(self.p_perrow)
            Popen(cmd.split(" "))

        rospy.sleep(30.0)
        if self.p_drawer:
            self.open_drawers()

        self.lock.release()
        self.prev_ros_time = current_ros_time
        self.run_enable.data = False
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = PrepareTask()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()