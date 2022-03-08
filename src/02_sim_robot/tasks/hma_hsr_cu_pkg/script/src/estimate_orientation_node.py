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
from libopencv import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_sim_lib_pkg") + "/script/src/lib")
from libhsrsub import *


# Globals
GP_LOOP_RATE = 30.0
GP_IMG_PATH = roslib.packages.get_pkg_dir("hma_hsr_cu_pkg") + "/io/orientation/"


class EstimateOrientation:
    """Orientation estimation."""

    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        libutil = LibUtil()
        libopencv = LibOpenCV()
        libhsrsub = LibHSRSub(libutil, libopencv)
        self.lib = {"util": libutil, "opencv": libopencv, "hsrsub": libhsrsub}

        self.template_imgs = {"cutlery":{"left":[], "right":[]}, "marker":{"left":[], "right":[]}}
        for path in glob.glob(GP_IMG_PATH + "cutlery/left_*.png"):
            cv_tmp = cv2.imread(path)
            self.template_imgs["cutlery"]["left"].append(cv_tmp)
        for path in glob.glob(GP_IMG_PATH + "cutlery/right_*.png"):
            cv_tmp = cv2.imread(path)
            self.template_imgs["cutlery"]["right"].append(cv_tmp)
        for path in glob.glob(GP_IMG_PATH + "marker/left_*.png"):
            cv_tmp = cv2.imread(path)
            self.template_imgs["marker"]["left"].append(cv_tmp)
        for path in glob.glob(GP_IMG_PATH + "marker/right_*.png"):
            cv_tmp = cv2.imread(path)
            self.template_imgs["marker"]["right"].append(cv_tmp)

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def main(self):
        """Main function."""
        if rospy.get_param("/sm_cu_node/oreint_item/flag", False):
            cv_rgb = self.lib["hsrsub"].get_hand_image()
            grasp_obj_id = rospy.get_param("/sm_cu_node/target_obj/id", "30")

            max_score = -1
            direction = "left"
            if grasp_obj_id in ["30", "31", "32"]:
                object = "Cutlery"
                for cv_tmp in self.template_imgs["cutlery"]["left"]:
                    result = cv2.matchTemplate(cv_rgb, cv_tmp, cv2.TM_CCOEFF_NORMED)
                    _, score, _, _ = cv2.minMaxLoc(result)
                    if max_score < score:
                        max_score = score
                        direction = "left"
                for cv_tmp in self.template_imgs["cutlery"]["right"]:
                    result = cv2.matchTemplate(cv_rgb, cv_tmp, cv2.TM_CCOEFF_NORMED)
                    _, score, _, _ = cv2.minMaxLoc(result)
                    if max_score < score:
                        max_score = score
                        direction = "right"
            else:
                object = "Marker"
                for cv_tmp in self.template_imgs["marker"]["left"]:
                    result = cv2.matchTemplate(cv_rgb, cv_tmp, cv2.TM_CCOEFF_NORMED)
                    _, score, _, _ = cv2.minMaxLoc(result)
                    if max_score < score:
                        max_score = score
                        direction = "left"
                for cv_tmp in self.template_imgs["marker"]["right"]:
                    result = cv2.matchTemplate(cv_rgb, cv_tmp, cv2.TM_CCOEFF_NORMED)
                    _, score, _, _ = cv2.minMaxLoc(result)
                    if max_score < score:
                        max_score = score
                        direction = "right"
                
            rospy.loginfo("[" + rospy.get_name() + "]: Object: %s, Direction: %s" % (object, direction))
            rospy.set_param("/sm_cu_node/oreint_item/direction", direction)

            # wait for the next
            while rospy.get_param("/sm_cu_node/oreint_item/flag", True):
                pass

        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = EstimateOrientation()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.main() 
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
