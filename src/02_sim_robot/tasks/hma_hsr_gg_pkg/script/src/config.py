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
import yaml

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

from hma_hsr_gg_pkg.msg import DetectDrivableAreaAction
from hma_hsr_gg_pkg.msg import DetectDrivableAreaGoal


GP_GRIPPER_OPEN_TIME = 4.0 # [sec]

GP_WAITING_POSES = [
    Pose2D(2.55, 0.3, 1.57),
    Pose2D(2.55, 1.6, 1.57)
]
GP_PLACE_POSE = Pose2D(2.55, 1.5, -1.57)
GP_SHELF_POSE = Pose2D(2.3, 3.8, 1.57)
GP_DELIVER_POSES = {
    "left": Pose2D(0.6, 3.0, 3.14),
    "right": Pose2D(0.6, 4.0, 3.14)
}

GP_COLLISION_MODELS = {
    "wrc_bookshelf": [Point(1.05, 2.55, 0), Point(0, 0, -1.57)],
    "wrc_frame": [Point(0, 0, 0), Point(0, 0, 0)]
}

GP_ARM_ROLL_POSES = {
    "front": np.deg2rad(0.0),
    "right": np.deg2rad(-90.0),
    "left": np.deg2rad(90.0),
}
GP_WRIST_FLEX_POSES = {
    "front": np.deg2rad(-110.0),
    "right": np.deg2rad(-110.0),
    "left": np.deg2rad(-110.0),
}
GP_X_OFFSETS = {
    "front": [-0.12, -0.03],
    "right": [0.0, 0.0],
    "left": [0.0, 0.0],
}
GP_Y_OFFSETS = {
    "front": [0.0, 0.0],
    "right": [0.1, 0.03],
    "left": [-0.1, -0.03],
}
GP_Z_OFFSETS = {
    "front": 0.01,
    "right": 0.02,
    "left": 0.02
}

GP_YCB_CONFIG = yaml.load(open(roslib.packages.get_pkg_dir("hma_yolact_pkg") + '/io/ycb_object_config.yaml'))["task2b"]
GP_YCB_NAMES = {}
for i, id in enumerate(GP_YCB_CONFIG["class_names"]):
    GP_YCB_NAMES[id] = GP_YCB_CONFIG["show_names"][i]