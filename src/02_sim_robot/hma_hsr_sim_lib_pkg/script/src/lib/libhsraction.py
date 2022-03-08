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

from hma_hsr_msgs.msg import Effort
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortActionGoal, GripperApplyEffortGoal, GripperApplyEffortActionFeedback, GripperApplyEffortActionResult


class LibHSRAction:
    """HSR ROS Action library for WRS simlator."""

    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        # ROS I/F
        self.ac_gripper_apply_force = actionlib.SimpleActionClient(
            "/hsrb/gripper_controller/apply_force", GripperApplyEffortAction)
        self.ac_gripper_apply_force.wait_for_server()

        return
    
    def delete(self):
        return

    def gripper_apply_force(self, force, is_sync=True):
        """Drive the gripper by force specification.

        Args:
            force (float): Grasping force.
                The value must be between 0.0 and 1.0.
            is_sync (bool, optional): Whether to wait for the end. Defaults to True.

        Returns:
            bool: If is_sync is True, returns the result of the action.
                Otherwise, it returns the SimpleActionClient.
        """
        goal = Effort()
        goal.effort = float(force)

        self.ac_gripper_apply_force.send_goal(goal)

        if is_sync:
            self.ac_gripper_apply_force.wait_for_result()
            return self.ac_gripper_apply_force.get_result()

        return self.ac_gripper_apply_force