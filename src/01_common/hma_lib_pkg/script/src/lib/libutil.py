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


class LibUtil:
    """Utility library."""
    
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        # ROS I/F
        self.p_descriptor_path = rospy.get_param(
            "/robot_descriptor",
            roslib.packages.get_pkg_dir("hma_common_pkg") + "/io/configs/descriptor.csv")

        return
    
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def get_robot_descriptor(self):
        """Load the robot's description file.

        Returns:
            dict: Description data.

        Examples:
            >>> print(get_robot_descriptor())
            {"TOPIC_LRF": "/hsrb/base_scan", "FRAME_MAP": "map", ...}
        """
        descriptor = {}

        f_descriptor = open(self.p_descriptor_path, "r")
        reader = csv.reader(f_descriptor)

        for row in reader:
            if len(row) != 2:
                rospy.logerr("[" + rospy.get_name() + "]: Descriptor is broken FAILURE")
                sys.exit(1)
            descriptor.update({str(row[0]):str(row[1])})
        f_descriptor.close()

        if len(descriptor) == 0:
            rospy.logerr("[" + rospy.get_name() + "]: Descriptor is broken FAILURE")
            sys.exit(1)

        return descriptor

    def rotate_coordinate(self, x, y, theta):
        """ Coordinate transformation of rotation.

        Args:
            x (float): x before rotation.
            y (float): y before rotation.
            theta (float): Rotation angle.

        Returns:
            float: x, y after rotation.
        """
        rot_x = x * np.cos(theta) - y * np.sin(theta)
        rot_y = x * np.sin(theta) + y * np.cos(theta)

        return rot_x, rot_y