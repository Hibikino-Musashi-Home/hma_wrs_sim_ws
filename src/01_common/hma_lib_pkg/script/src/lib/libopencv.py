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


class LibOpenCV:
    """OpenCV library."""
    
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        self.cb = CvBridge()
        self.LUT = []

        return
    
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def smi2cv(self, smi, encode, is_depth = False):
        """Convert from sensor_msgs/Image to OpenCV image.

        Args:
            smi (sensor_msgs/Image): Image before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.
            is_depth (bool, optional): Whether to convert depth image. Defaults to False.

        Returns:
            ndarray: Converted OpenCV image.
            If False is returned, the conversion has failed.
        """
        try:
            if is_depth:
                cv_tmp = self.cb.imgmsg_to_cv2(smi, encode)
                cv = np.array(cv_tmp, dtype=np.uint16)
            else:
                cv = self.cb.imgmsg_to_cv2(smi, encode)
            return cv
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def smci2cv(self, smci):
        """Convert from sensor_msgs/CompressedImage to OpenCV image.

        Args:
            smci (sensor_msgs/CompressedImage): CompressedImage before conversion.

        Returns:
            ndarray: Converted OpenCV image.
            If False is returned, the conversion has failed.
        """
        try:
            np_arr = np.fromstring(smci.data, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def cv2smi(self, cv, encode):
        """Convert from OpenCV image to sensor_msgs/Image.

        Args:
            cv (ndarray): OpenCV image before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.

        Returns:
            sensor_msgs/Image: converted Image.
            If False is returned, the conversion has failed.
        """
        try:
            return self.cb.cv2_to_imgmsg(cv, encode)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def smci2smi(self, smci, encode):
        """Convert from sensor_msgs/CompressedImage to sensor_msgs/Image.

        Args:
            smci (sensor_msgs/CompressedImage): CompressedImage before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.

        Returns:
            sensor_msgs/Image: converted Image.
            If False is returned, the conversion has failed.
        """
        try:
            np_arr = np.fromstring(smci.data, np.uint8)
            cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return self.cb.cv2_to_imgmsg(cv, encode)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False