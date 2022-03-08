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
import smach
import smach_ros
import inspect
import dynamic_reconfigure.client

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

import config
from config import *


class Init(smach.State):
    """Initialize state."""
    
    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor
        return

    def __del__(self):
        return 

    def execute(self, userdata):
        rospy.set_param(rospy.get_name() + "/obj_place", 0)
        rospy.set_param(rospy.get_name() + "/obj_id", "-1")
        rospy.set_param(rospy.get_name() + "/table_id", 0)
        rospy.set_param(rospy.get_name() + "/grasp/loop", 0)
        return "next"


class Wait4Start(smach.State):
    """Wait for start state."""
    
    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        # ROS I/F
        self.p_wait_time = rospy.get_param(rospy.get_name() + "/start_wait_time", 0.0)

        return

    def __del__(self):
        return

    def execute(self, userdata):
        self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
        rospy.loginfo("[" + rospy.get_name() + "]: Let's GO!!")
        return "next"


class Start(smach.State):
    """Start state."""
    
    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor
        return

    def __del__(self):
        return

    def execute(self, userdata):
        return "next"


class GoToRecPlace(smach.State):
    """Go to recognition place state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor
        return

    def __del__(self):
        return

    def execute(self, userdata):
        ms_cfg = {"start_pose": "auto"}
        self.lib["hsrmoveit"].set_base_acc(0.5)
        self.lib["hsrmoveit"].base_go_abs(GP_WAITING_POSES[0],
                                          motion_synthesis_config=ms_cfg)
                                          
        ms_cfg["goal_pose"] = {"arm_lift_joint": 0.0,
                               "arm_flex_joint": 0.0,
                               "arm_roll_joint": np.deg2rad(-90.0),
                               "wrist_flex_joint": np.deg2rad(-110.0),
                               "wrist_roll_joint": 0.0,
                               "head_pan_joint": 0.0,
                               "head_tilt_joint": np.deg2rad(-50.0)}
        self.lib["hsrmoveit"].base_go_abs(GP_WAITING_POSES[1],
                                          motion_synthesis_config=ms_cfg)

        return "next"


class EnterToRoom2(smach.State):
    """Enter to Room 2 state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "loop", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        # ROS I/F
        self.pub_finish = rospy.Publisher(
            "/detect_drivable_area_server/finish", Empty, queue_size = 1)

        self.ac_detect_drivable_area = actionlib.SimpleActionClient(
            "/detect_drivable_area", DetectDrivableAreaAction)
        # self.ac_detect_drivable_area.wait_for_server()

        self.client_global_costmap = dynamic_reconfigure.client.Client(
            "/hma_move_base/global_costmap", timeout = 10)
        self.client_global_costmap_inflater = dynamic_reconfigure.client.Client(
            "/hma_move_base/global_costmap/inflater", timeout = 10)
        self.client_local_costmap = dynamic_reconfigure.client.Client(
            "/hma_move_base/local_costmap", timeout = 10)
        self.client_local_costmap_inflater = dynamic_reconfigure.client.Client(
            "/hma_move_base/local_costmap/inflater", timeout = 10)

        return

    def __del__(self):
        return

    def af_detect_drivable_area(self, frame, transform_pose):
        goal = DetectDrivableAreaGoal()
        goal.frame = str(frame)
        goal.transform_pose = transform_pose

        self.ac_detect_drivable_area.send_goal(goal)
        self.ac_detect_drivable_area.wait_for_result()
        return self.ac_detect_drivable_area.get_result()

    def execute(self, userdata):
        # drivable area detection
        transform_pose = self.lib["tf"].get_pose_without_offset(
            self.robot_descriptor["FRAME_MAP"], self.robot_descriptor["FRAME_RGBD_POINT"])
        result = self.af_detect_drivable_area(self.robot_descriptor["FRAME_MAP"], transform_pose)
        rospy.sleep(1.0)

        # navigation
        ms_cfg = {"goal_pose": {"arm_lift_joint": 0.2,
                                "arm_flex_joint": 0.0,
                                "arm_roll_joint": np.deg2rad(-90.0),
                                "wrist_flex_joint": np.deg2rad(-90.0),
                                "wrist_roll_joint": 0.0,
                                "head_pan_joint": 0.0,
                                "head_tilt_joint": np.deg2rad(-20.0)}}
        robot_radius = rospy.get_param("/detect_drivable_area_server/robot_radius", 0.3)
        while not rospy.is_shutdown():
            result = self.lib["hsrnav"].go_abs(GP_SHELF_POSE, 
                                               motion_synthesis_config=ms_cfg)
            if result:
                break

            robot_radius -= 0.01
            self.client_global_costmap.update_configuration({
                "robot_radius": robot_radius})
            self.client_global_costmap_inflater.update_configuration({
                "inflation_radius": robot_radius})
            self.client_local_costmap.update_configuration({
                "robot_radius": robot_radius})
            self.client_local_costmap_inflater.update_configuration({
                "inflation_radius": robot_radius})
            rospy.sleep(0.1)

        self.pub_finish.publish(Empty())

        return "next"


class GoToShelf(smach.State):
    """Go to shelf state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor
        return

    def __del__(self):
        return

    def execute(self, userdata):
        ms_cfg = {"start_pose": "auto",
                  "goal_pose": {"arm_lift_joint": 0.2,
                                "arm_flex_joint": 0.0,
                                "arm_roll_joint": np.deg2rad(-90.0),
                                "wrist_flex_joint": np.deg2rad(-90.0),
                                "wrist_roll_joint": 0.0,
                                "head_pan_joint": 0.0,
                                "head_tilt_joint": np.deg2rad(-20.0)}}
        self.lib["hsrnav"].go_abs(GP_SHELF_POSE,
                                  motion_synthesis_config=ms_cfg)

        return "next"


class GetObject(smach.State):
    """Get object from the shelf state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "move", "loop", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        self.obj_id = -1
        self.person = ""
        self.done = False

        # ROS I/F
        self.sub_message = rospy.Subscriber(
            "/message",
            String,
            self.subf_message,
            queue_size = 1
        )

        return

    def __del__(self):
        return

    def subf_message(self, message):
        rospy.logwarn("[" + rospy.get_name() + "]: Received: " + str(message.data))

        if "done" not in message.data:
            self.obj_id = [int(k) for k, v in GP_YCB_NAMES.items() if v in message.data][0]
            rospy.loginfo("[" + rospy.get_name() + "]: Food: " + str(GP_YCB_NAMES[str(self.obj_id)]))

            self.person = "left" if "left" in message.data else "right"
            rospy.loginfo("[" + rospy.get_name() + "]: Person: " + str(self.person))
            rospy.set_param(rospy.get_name() + "/person", self.person)
        else:
            rospy.loginfo("[" + rospy.get_name() + "]: Finish.")
            self.done = True

        return

    def execute(self, userdata):
        if self.done:
            sys.exit()

        if self.obj_id == -1 or self.person == "":
            rospy.logwarn("[" + rospy.get_name() + "]: Do not received /message.")
            # return "except"

        is_check = False
        while not rospy.is_shutdown():
            self.lib["hsraction"].gripper_apply_force(1.0)

            index = -1
            if self.obj_id != -1:
                for cnt in range(10):
                    yolact_with_pose = self.lib["yolact"].recognize().with_pose
                    for i in range(yolact_with_pose.amount):
                        if yolact_with_pose.is_valid_pose[i] is False:
                            continue
                        if yolact_with_pose.id[i] != str(self.obj_id + 1):
                            continue
                        index = i
                        is_check = True
                        break
                    if index != -1:
                        break

            # grasp the object closest to the robot
            if index == -1:
                rospy.logwarn("[" + rospy.get_name() + "]: Cannot find the designated object.")

                yolact_with_pose = self.lib["yolact"].recognize().with_pose
                min_distance = np.inf
                for i in range(yolact_with_pose.amount):
                    if yolact_with_pose.is_valid_pose[i] is False:
                        continue

                    pose_base = self.lib["tf"].get_pose_with_offset(
                        self.robot_descriptor["FRAME_BASE_FOOTPRINT"],
                        self.robot_descriptor["FRAME_RGBD_POINT"],
                        "base_" + str(i),
                        yolact_with_pose.pose[i])

                    distance_x, distance_y, distance_z = pose_base.position.x, pose_base.position.y, pose_base.position.z
                    distance = math.sqrt(pow(distance_x, 2) + pow(distance_y, 2))
                    if distance > min_distance:
                        continue
                    min_distance = distance
                    index = i
            
            if index == -1:
                rospy.logwarn("[" + rospy.get_name() + "]: Cannot find the object.")
                return "move"

            # grasp pose estimation
            self.loop_cnt = 0
            while not rospy.is_shutdown():
                cv_mask = self.lib["opencv"].smi2cv(yolact_with_pose.mask, "mono8")
                cv_mask_obj = np.zeros(cv_mask.shape, np.uint8)
                cv_mask_obj[:, :] = np.where(cv_mask == index+1, 255, 0)

                grasp_pose = self.lib["action"].grasp_pose_estimation(
                    "odom", self.robot_descriptor["FRAME_RGBD_POINT"],
                    yolact_with_pose.id[index],
                    yolact_with_pose.d,
                    self.lib["opencv"].cv2smi(cv_mask_obj, "mono8"),
                    GraspPose.FRONT)

                # transform_pose = self.lib["tf"].get_pose_without_offset(
                #     "odom", self.robot_descriptor["FRAME_RGBD_POINT"])
                # grasp_pose = self.lib["action"].grasp_pose_estimation(
                #     "odom", yolact_with_pose, transform_pose, index, GraspPose.FRONT)
                # self.lib["tf"].send_transform("grasp", "odom", grasp_pose.pose)

                pose_rec = self.lib["tf"].get_pose_with_offset(
                    "odom", self.robot_descriptor["FRAME_RGBD_POINT"], "rec", yolact_with_pose.pose[index])
                
                def is_ok(before, after, tolerance):
                    if abs(after.x - before.x) < tolerance:
                        if abs(after.y - before.y) < tolerance:
                            return True

                if is_ok(pose_rec.position, grasp_pose.pose.position, 0.05):
                    break

                if self.loop_cnt > 10:
                    return "move"
                self.loop_cnt += 1
            
            # add obstacles from world model
            # self.lib["hsrmoveit"].delete()
            # for key in GP_COLLISION_MODELS.keys():
            #     self.lib["hsrmoveit"].add_model_from_sdf(GP_COLLISION_MODELS[key][0], GP_COLLISION_MODELS[key][1],
            #         roslib.packages.get_pkg_dir("hma_hsr_sim_pkg") + "/io/models/" + str(key) + "/model.sdf")

            # check grasp pose
            pose_check = self.lib["tf"].get_pose_with_offset(
                self.robot_descriptor["FRAME_MAP"],
                self.robot_descriptor["FRAME_RGBD_POINT"],
                "pose_check",
                yolact_with_pose.pose[index])

            posture = "front"
            if pose_check.position.x >= 2.5:
                posture = "right"
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
            elif pose_check.position.x <= 2.0:
                posture = "left"
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
                
            joints = {"arm_flex_joint": 0.0,
                      "arm_roll_joint": GP_ARM_ROLL_POSES[posture],
                      "wrist_flex_joint": GP_WRIST_FLEX_POSES[posture],
                      "wrist_roll_joint": 0.0}
            self.lib["hsrpub"].whole_body_move_to_arm_positions(joints, 0.1)
            rospy.sleep(1.0)

            offset_x = GP_X_OFFSETS[posture][0]
            offset_y = GP_Y_OFFSETS[posture][0]
            offset_z = GP_Z_OFFSETS[posture]
            width = grasp_pose.d

            if width < 0.05:
                offset_x -= 0.02

            # grasp
            end_effector_q = self.lib["tf"].get_pose_with_offset(
                "odom", "hand_palm_link", "end_effector_pose",
                Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
            end_effector_rpy = self.lib["tf"].quaternion2euler(end_effector_q)

            base_link_q = self.lib["tf"].get_pose_with_offset(
                "odom", "base_link", "base_link_pose",
                Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
            base_link_yaw = self.lib["tf"].quaternion2euler(base_link_q)[2]

            stats = -1
            pos = grasp_pose.pose.position
            offset_x0, offset_y0 = self.lib["util"].rotate_coordinate(offset_x, offset_y, base_link_yaw)
            if width < 0.05:
                if posture == "front":
                    offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(GP_X_OFFSETS[posture][1] - 0.02, GP_Y_OFFSETS[posture][1], base_link_yaw)
                elif posture == "right":
                    offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(GP_X_OFFSETS[posture][1], GP_Y_OFFSETS[posture][1] + 0.02, base_link_yaw)
                else:
                    offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(GP_X_OFFSETS[posture][1], GP_Y_OFFSETS[posture][1] - 0.02, base_link_yaw)
            else:
                offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(GP_X_OFFSETS[posture][1], GP_Y_OFFSETS[posture][1], base_link_yaw)

            cnt = 0
            failure_flag = False
            while stats < 0:
                if cnt >= 3:
                    failure_flag = True
                    break
                cnt += 1

                stats = self.lib["hsrmoveit"].whole_body_IK_cartesian(
                    [pos.x + offset_x0],
                    [pos.y + offset_y0],
                    [pos.z + offset_z],
                    [end_effector_rpy[0]],
                    [end_effector_rpy[1]],
                    [end_effector_rpy[2]],
                    eef_step = 0.1)
                rospy.sleep(1.0)
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))

                stats = self.lib["hsrmoveit"].whole_body_IK_cartesian(
                    [pos.x + offset_x1],
                    [pos.y + offset_y1],
                    [pos.z + offset_z],
                    [end_effector_rpy[0]],
                    [end_effector_rpy[1]],
                    [end_effector_rpy[2]],
                    eef_step = 0.1)

            if not failure_flag:
                break

            self.lib["hsrmoveit"].arm_neutral()

            ms_cfg = {"start_pose": "auto",
                      "goal_pose": {"arm_lift_joint": 0.2,
                                    "arm_flex_joint": 0.0,
                                    "arm_roll_joint": np.deg2rad(-90.0),
                                    "wrist_flex_joint": np.deg2rad(-90.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": np.deg2rad(-20.0)}}
            self.lib["hsrnav"].go_abs(GP_SHELF_POSE,
                                      motion_synthesis_config=ms_cfg)

        if grasp_pose.id == "12": # banana
            self.lib["hsraction"].gripper_apply_force(1.0)
            rospy.sleep(1.0)
        else:
            self.lib["hsraction"].gripper_apply_force(1.0)
            rospy.sleep(0.5)
            currents = self.lib["hsrsub"].get_joint_positions()
            if currents["hand_motor_joint"] < 0.1:
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(-30.0), wait=False, time=2.0)

        # raise the arm
        currents = self.lib["hsrsub"].get_joint_positions()
        currents["arm_lift_joint"] += 0.03
        currents["arm_flex_joint"] += np.deg2rad(10.0)
        self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 1.0)
        rospy.sleep(1.0)

        # backward
        sec = 2.0
        for i in xrange(int(sec / 0.1)):
            self.lib["hsrpub"].omni_base_vel(-0.2, 0, 0)
            rospy.sleep(0.1)

        # check if the object is grasped
        if is_check:
            ms_cfg = {"start_pose": "auto",
                      "goal_pose": {"arm_lift_joint": 0.2,
                                    "arm_flex_joint": 0.0,
                                    "arm_roll_joint": np.deg2rad(-90.0),
                                    "wrist_flex_joint": np.deg2rad(-90.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": np.deg2rad(-20.0)}}
            self.lib["hsrnav"].go_abs(GP_SHELF_POSE, 
                                      motion_synthesis_config=ms_cfg)

            rec_res = self.lib["yolact"].recognize()
            yolact_with_pose = rec_res.with_pose
            _index = -1
            for i in range(yolact_with_pose.amount):
                if yolact_with_pose.id[i] != str(self.obj_id + 1):
                    continue
                _index = i
                break

            if _index != -1:
                return "move"

        return "next"


class DeliverObject(smach.State):
    """Deliver the object to designated person."""
    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor
        return

    def __del__(self):
        return

    def execute(self, userdata):
        person = rospy.get_param(rospy.get_name() + "/person", "left")
        ms_cfg = {"start_pose": "auto",
                  "goal_pose": {"arm_lift_joint": 0.0,
                                "arm_flex_joint": np.deg2rad(-90.0),
                                "arm_roll_joint": 0.0,
                                "wrist_flex_joint": 0.0,
                                "wrist_roll_joint": 0.0,
                                "head_pan_joint": 0.0,
                                "head_tilt_joint": 0.0}}
        stats = self.lib["hsrnav"].go_abs(GP_DELIVER_POSES[person],
                                          motion_synthesis_config=ms_cfg)

        # relese object
        self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
        rospy.sleep(3.0)

        return "next"


class End(smach.State):
    """End state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["end"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        return

    def __del__(self):
        return

    def execute(self, userdata):
        return "end"


class Except(smach.State):
    """Except state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        return

    def __del__(self):
        return

    def execute(self, userdata):
        return "except"
