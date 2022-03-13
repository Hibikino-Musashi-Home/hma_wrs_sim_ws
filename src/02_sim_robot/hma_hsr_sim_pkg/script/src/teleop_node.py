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

import pygame
from pygame.locals import *

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

sys.path.append(roslib.packages.get_pkg_dir("hma_hsr_sim_lib_pkg") + "/script/src/lib")
from libhsrpub import *


# Global
GP_LOOP_RATE = 100.0


class Teleop:
    """Teleoperation for HSR Sim using pygame."""
    
    def __init__(self, run_enable=True):
        """Constructor.

        Args:
            run_enable (bool, optional): Run enable. Defaults to True.
        """
        
        self.lock = threading.Lock()
        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        libhsrpub = LibHSRPub()
        self.lib = {"hsrpub": libhsrpub}

        self.run_enable = Bool(run_enable)

        # ROS I/F
        self.sub_run_enable = rospy.Subscriber(rospy.get_name() + "/run_enable",
                                               Bool,
                                               self.subf_run_enable,
                                               queue_size = 1)

        # pygame
        w, h = 200, 200
        x, y = w / 2.0, h / 2.0
        pygame.init()
        pygame.display.set_mode((w, h), 0, 32)
        pygame.display.set_caption("Teleop")
        self.screen = pygame.display.get_surface()

        rospy.sleep(0.1)
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

    def get_speed(self, pressed, key_plus, key_minus):
        """Get speed.

        Args:
            pressed (list[bool]): List of keys pressed or not (pygame format).
            key_plus (int): Key that returns positive velocity.
            key_minus (int): Key that returns negative velocity.

        Returns:
            float: Speed.
        """
        speed = 0.0
        if pressed[key_plus]:
            speed = 1.0
        elif pressed[key_minus]:
            speed = -1.0
        return speed

    def main(self):
        """Main function."""
        self.screen.fill((0, 0, 0))
        pygame.display.update()

        pressed = pygame.key.get_pressed()
        pygame.event.poll()
        if sum(pressed) == 0:
            return

        x = self.get_speed(pressed, pygame.K_w, pygame.K_s) # w: forward, s: backward
        y = self.get_speed(pressed, pygame.K_a, pygame.K_d) # a: left, d: right
        yaw = self.get_speed(pressed, pygame.K_q, pygame.K_e) # q: rotate left, e: rotate right

        self.lib["hsrpub"].omni_base_vel(
            x * 0.2, y * 0.2, yaw * 0.5)

        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = Teleop()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.main()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()