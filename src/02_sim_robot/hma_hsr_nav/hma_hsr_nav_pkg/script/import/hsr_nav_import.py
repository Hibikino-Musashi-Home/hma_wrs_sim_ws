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

"""HSR navigation import

Importing commonly used HSR navigation.
"""

"""Imports."""
import numpy as np

from actionlib_msgs.msg import GoalID, GoalStatus
from nav_msgs.srv import GetPlan
from tmc_omni_path_follower.msg import PathFollowerAction
from tmc_omni_path_follower.msg import PathFollowerActionGoal

from hma_hsr_nav_action.msg import MotionSynthesisAction
from hma_hsr_nav_action.msg import MotionSynthesisGoal
from hma_hsr_nav_action.msg import MotionSynthesisFeedback
from hma_hsr_nav_action.msg import MotionSynthesisResult

from hma_hsr_nav_action.msg import OmniPathFollowerAction
from hma_hsr_nav_action.msg import OmniPathFollowerGoal
from hma_hsr_nav_action.msg import OmniPathFollowerFeedback
from hma_hsr_nav_action.msg import OmniPathFollowerResult


"""Globals"""
GP_MIN_LINEAR_VEL  = 0.0 # Minimum linear speed
GP_MAX_LINEAR_VEL  = 0.4 # Maximum linear speed
GP_MIN_ANGULAR_VEL = 0.0 # Minimum angular speed
GP_MAX_ANGULAR_VEL = 1.5 # Maximum angular speed

from enum import IntEnum
class NavState(IntEnum):
    INIT = -1
    START = 0
    ACTIVE = 1
    SUCCESS = 2
    FAILURE = 3

class PathFollow(IntEnum):
    START = 0
    CANCEL = 1

