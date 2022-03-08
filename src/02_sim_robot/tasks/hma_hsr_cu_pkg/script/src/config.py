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


GP_GRIPPER_OPEN_TIME = 4.0 # [sec]

GP_DRAWER_POSES = [
    Pose2D(0.08, 0.3, -1.57),
    Pose2D(0.08, 0.25, -1.57),
    Pose2D(0.43, 0.25, -1.57),
]
GP_HANDLE_POSES = [
    Point(0.14, -0.36, 0.3),
    Point(0.14, -0.38, 0.535),
    Point(0.49, -0.43, 0.29),
]
GP_BACK_VAL = [-0.39, -0.39, -0.38]

GP_MAPPING_POSE = {
    "floor": [Pose2D(0.0, 0.2, 1.57), Pose2D(1.0, 0.2, 1.57)],
    "table": Pose2D(0.7, 0.9, 1.57)
}

GP_REC_Y = 0.9
GP_REC_POSES =[
    Pose2D(0.1, 0.5, 1.57),
    Pose2D(0.7, 0.5, 1.57),
    Pose2D(1.3, 0.5, 1.57),
    Pose2D(1.0, 0.9, 1.57),
    Pose2D(0.6, 0.9, 1.57),
    Pose2D(0.0, 0.9, 1.57)
]
GP_REC_JOINTS = {
    "arm_lift_joint":[0.0, 0.0, 0.0, 0.2, 0.2, 0.2],
    "head_tilt_joint":[-50.0, -50.0, -50.0, -30.0, -30.0, -20.0]
}

GP_DEPOSIT_POSES = {
    "tool": Pose2D(0.10, 0.32, -1.57), # Drawer top, bottom: Tools
    "shape": Pose2D(0.43, 0.32, -1.57), # Drawer left: Shape items
    "kitchen": [Pose2D(0.96, 0.25, -1.57), Pose2D(0.96, -0.14, -1.57)], # Container A: Kitchen items
    "orientation": Pose2D(1.3, 0.15, -1.57), # Container B: Orientation items
    "food": [
        Pose2D(1.52, 0.0, -1.57), # Tray A: Food
        Pose2D(1.77, 0.0, -1.57) # Tray B: Food
    ],
    "task": Pose2D(2.3, -0.1, -1.57) # Bin A: Task items
}
GP_CATEGORY_ID_RANGES = {
    "tool": {"min": 35, "max": 52},
    "shape": {"min": 53, "max": 69},
    "kitchen": {"min": 19, "max": 34},
    "food": {"min": 1, "max": 18},
    "task": {"min": 70, "max": 82}
}

GP_CONTAINER_POSE = Point(1.35, -0.54, 0.6)

GP_COLLISION_MODELS = {
    "wrc_tall_table": [Point(-0.29, 1.2, -0.05), Point(0, 0, 0)],
    "wrc_long_table": [Point(-0.2, -0.33, -0.05), Point(0, 0, -1.57)],
    "wrc_frame": [Point(0, 0, 0), Point(0, 0, 0)]
}

GP_NON_DIRECTION_OBJ_IDS = ["12", "13", "15", "17", "18", "24", "25", "29", "54", "55", "56", "57", "58", "62", "63", "65"]

GP_YCB_CONFIG = yaml.load(open(roslib.packages.get_pkg_dir("hma_yolact_pkg") + '/io/ycb_object_config.yaml'))["task1"]
GP_YCB_NAMES = {}
for i, id in enumerate(GP_YCB_CONFIG["class_names"]):
    GP_YCB_NAMES[id] = GP_YCB_CONFIG["show_names"][i]