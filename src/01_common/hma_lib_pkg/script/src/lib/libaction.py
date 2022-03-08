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


class LibAction:
    """ROS Action library."""

    def __init__(self, libtf):
        """Constractor.

        Args:
            libtf (instance): TF library instance.
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {"tf": libtf}

        # ROS I/F
        self.ac_grasp_pose_estimation = actionlib.SimpleActionClient(
            "/grasp_pose_estimation", GraspPoseEstimationAction)
        #self.ac_grasp_pose_estimation.wait_for_server()

        self.ac_mapping_object = actionlib.SimpleActionClient(
            "/mapping_object", MappingObjectAction)
        #self.ac_mapping_object.wait_for_server()

        return
        
    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def grasp_pose_estimation(self,
                              conv_frame,
                              origin_frame,
                              id,
                              d,
                              mask,
                              specific_direction):
        """Function of estimate grasp pose from YOLACT results.

        Args:
            conv_frame (str): Name of the converted frame.
            origin_frame (str): Name of the original frame.
            id (str): Object ID.
            d (sensor_msgs/Image): Depth image.
            mask (sensor_msgs/Image): Mask image of the object to grasp pose estimation.
            specific_direction (int): Fix the grasping direction to be estimated.
                Select from GraspPose.[TOP, FRONT].
                TOP: Estimate the grasp pose only from the top.
                FRONT: Estimate the grasp pose only from the front.

        Returns:
            hma_common_action/GraspPoseEstimationResult: Results of grasp pose estimation.
        """
        goal = GraspPoseEstimationGoal()
        goal.frame = str(conv_frame)
        goal.id = str(id)
        goal.d = d
        goal.mask = mask
        goal.transform_pose = self.lib["tf"].get_pose_without_offset(
            conv_frame,
            origin_frame)
        goal.specific_direction = specific_direction

        self.ac_grasp_pose_estimation.send_goal(goal)
        self.ac_grasp_pose_estimation.wait_for_result()

        return self.ac_grasp_pose_estimation.get_result()

    def mapping_object(self,
                       action,
                       id = [""],
                       name = [""],
                       pose = [Pose()],
                       score = [0.0],
                       search_place = ""):
        """Function of mapping cluttered objects.

        Args:
            action (int): Select the action to be executed.
                Select from MappingAction.[ADD, MAPPING, DELETE, DELETEALL, CHECK, GET, GETALL, GETNEAREST]
                ADD: Recognize and add objects.
                MAPPING: Run the mapping and save the data.
                DELETE: Delete specific object from the mapping data.
                DELETEALL: Delete all objects from the mapping data.
                CHECK: Check for the existence of an object in the mapping data, and if it exists, 
                    compare the scores and return the higher one.
                GET: Get a specific object from the mapping data.
                GETALL: Get all objects from the mapping data.
                GETNEAREST: Get nearest object from the mapping data.
            id (list[str], optional): object ID.
                Required for ADD, CHECK. Defaults to "".
            name (list[str], optional): object name.
                Required for ADD. Defaults to "".
            pose (list[geometry_msgs/Pose], optional): 3D coordinates of a specific object.
                Required for ADD, DELETE, CHECK, GETNEAREST. Defaults to Pose().
            score (list[float], optional): Recognition score of a comparison objects.
                Required for ADD, CHECK. Defaults to 0.0.
            search_place (str, optional): Limit the search area.
                Optional for ADD, DELETE, CHECK, GETNEAREST. Defaults to "".

        Returns:
            hma_common_action/MappingObjectResult: Results for each action.
        """
        goal = MappingObjectGoal()
        goal.action = action
        goal.id = id if isinstance(id, list) else [id]
        goal.name = name if isinstance(name, list) else [name]
        goal.pose = pose if isinstance(pose, list) else [pose]
        goal.score = score if isinstance(score, list) else [score]
        goal.search_place = search_place

        self.ac_mapping_object.send_goal(goal)
        self.ac_mapping_object.wait_for_result()

        return self.ac_mapping_object.get_result()