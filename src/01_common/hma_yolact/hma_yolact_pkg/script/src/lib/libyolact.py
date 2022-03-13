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
import rospy
import roslib
import actionlib
import yaml

from sensor_msgs.msg import Image

from hma_yolact_action.msg import YolactAction, YolactGoal

import os

class LibYolact:
    """YOLACT ROS Action library."""

    def __init__(self, is_init=True, action_name="/yolact"):
        """Constructor.

        Args:
            is_is_initinit_rec (bool, optional): _description_. Defaults to True.
            action_name (str, optional): _description_. Defaults to "/yolact".
        """
        # ROS I/F
        self._ac_yolact = actionlib.SimpleActionClient(action_name, YolactAction)
        self._ac_yolact.wait_for_server()

        # Initialize OpenCL runtime
        if is_init:
            self.recognize()

        return

    def get_class_cfg(self, config_name):
        """Get class config.

        Args:
            config_name (str): Config name.

        Returns:
            dict[str, str]: Class config.
                class_names: Class IDs.
                show_names: Class labels.

        Examples:
            >>> print(get_class_cfg["config_name"])
            {"class_names": ("1", "2", ...), "show_names": ("class_1", "class_2", ...)}
        """
        ycb_config = yaml.load(open(roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/io/ycb_object_config.yaml"))
        return ycb_config[config_name]

    def recognize(self,
                  rgb=Image(),
                  d=Image(),
                  max_distance=-1.0,
                  specific_id="",
                  show_name=False):
        """Object recognition using YOLACT.

        Args:
            rgb (sensor_msgs/Image, optional): Specify if you want to recognize a specific RGB image. Defaults to Image()
            d (sensor_msgs/Image, optional): Used to obtain the distance. Defaults to Image().
            max_distance (float, optional): The maximum recognition distance can be specified.
                If a negative value is given, it is unlimited. Defaults to -1.0[m].
            specific_id (str, optional): Only specified objects can be recognized. Defaults to "".
            show_name (bool, optional): Display by object name, not by ID. Defaults to False.

        Returns:
            hma_yolact_action/YolactResult: Recognition results.
        """
        goal = YolactGoal()
        goal.rgb = rgb
        goal.d = d
        goal.max_distance = max_distance
        goal.specific_id = specific_id
        goal.show_name = show_name

        self._ac_yolact.send_goal(goal)
        self._ac_yolact.wait_for_result()
        return self._ac_yolact.get_result()