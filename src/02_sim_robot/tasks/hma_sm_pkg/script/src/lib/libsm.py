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

import inspect

import tty
import termios

class LibSM:
    """StateMachine library."""

    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {}

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def get_key_without_buffer(self):
        """Get key input without buffer.

        Returns:
            str: Input key.
        """
        buf_fd = sys.stdin.fileno()
        buf_setting = termios.tcgetattr(buf_fd)

        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(buf_fd, termios.TCSADRAIN, buf_setting)

        return key

    def get_class_names(self, obj):
        """Get class names of object

        Args:
            obj (module): Object.

        Returns:
            list[str]: Class names.
        """
        class_names = map(lambda x:x[0], inspect.getmembers(obj, inspect.isclass))
        return class_names

    def sort_class_names(self, obj, class_names):
        """Sort class names in order of description.

        Args:
            obj (module): Object.
            class_names (str): Class names.

        Returns:
            list[str]: Sorted class names.
        """
        lineno_and_class_names = []
        lineno_and_class_names.append([0, "Default"])
        for i in xrange(len(class_names)):
            attr = getattr(obj, class_names[i])
            if attr.__module__ == obj.__name__:
                class_instance = attr()
                lineno_and_class_names.append([class_instance.lineno, class_names[i]])
                del class_instance

        lineno_and_class_names = sorted(lineno_and_class_names, key = lambda x:x[0])
        sorted_class_names = []
        for i in xrange(len(lineno_and_class_names)):
            sorted_class_names.append(lineno_and_class_names[i][1])

        return sorted_class_names

    def select_start_state(self, sorted_class_names):
        """Select start state.

        Args:
            sorted_class_names (list[str]): Sorted class names.

        Returns:
            str: Start state name.
        """
        start_state = sorted_class_names[0]
        while not rospy.is_shutdown():
            for i in xrange(len(sorted_class_names)):
                sys.stdout.write("\r{0:^20}".format(sorted_class_names[i]))
                sys.stdout.flush()

                while not rospy.is_shutdown():
                    key = self.get_key_without_buffer()
                    if key.encode("hex") in [b"0d", b"20", b"64"]: #Enter/Space/d
                        break
                if key.encode("hex") in [b"0d"]: #Enter
                    start_state = sorted_class_names[i]
                    break
                if key.encode("hex") in [b"64"]: #d
                    break
            else:
                continue

            print("\n")
            return start_state

    def get_start_state(self, obj):
        """Get start state.

        Args:
            obj (module): Object.

        Returns:
            str: Start state name.
        """
        class_names = self.get_class_names(obj)
        sorted_class_names = self.sort_class_names(obj, class_names)
        start_state = self.select_start_state(sorted_class_names)
        return start_state
