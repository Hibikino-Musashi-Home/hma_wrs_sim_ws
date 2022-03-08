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

"""ROS import

Importing commonly used ROS libraries and messages.
"""

"""Imports."""
import rospy
import rospkg
import roslib
import actionlib
import message_filters

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import Int64
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Header

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import JointState
from sensor_msgs import point_cloud2

from cv_bridge import CvBridge

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import WrenchStamped

from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseFeedback
from move_base_msgs.msg import MoveBaseResult

from actionlib_msgs.msg import GoalStatusArray

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import InteractiveMarkerControl

from hma_common_action.msg import GraspPoseEstimationAction
from hma_common_action.msg import GraspPoseEstimationGoal
from hma_common_action.msg import GraspPoseEstimationFeedback
from hma_common_action.msg import GraspPoseEstimationResult

from hma_common_action.msg import MappingObjectAction
from hma_common_action.msg import MappingObjectGoal
from hma_common_action.msg import MappingObjectFeedback
from hma_common_action.msg import MappingObjectResult

from hma_yolact_msgs.msg import YolactWithoutPose
from hma_yolact_msgs.msg import YolactWithPose