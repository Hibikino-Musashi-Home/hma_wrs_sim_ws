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
        rospy.set_param(rospy.get_name() + "/no_rec", False)
        rospy.set_param(rospy.get_name() + "/oreint_item/flag", False)
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
        rospy.loginfo("[" + rospy.get_name() + "]: Wait for start: " + str(self.p_wait_time) + " sec")
        rospy.sleep(float(self.p_wait_time))
        rospy.loginfo("[" + rospy.get_name() + "]: Let's GO!!")
        return "next"

class Start(smach.State):
    """Start state."""
    
    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        # ROS I/F
        self.pub_time_supervisor = rospy.Publisher(
            "/manage_task_time_node/run_enable", Bool, queue_size=1)

        return

    def __del__(self):
        return

    def execute(self, userdata):
        # start task timer
        self.pub_time_supervisor.publish(Bool(True))
        return "next"


class OpenDrawer(smach.State):
    """Open drawer state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        self.p_open_drawer = rospy.get_param(
            rospy.get_name() + "/open_drawer", True)

        return

    def __del__(self):
        return

    def execute(self, userdata):
        if not self.p_open_drawer:
            return "next"

        for i in range(len(GP_HANDLE_POSES)):
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(20.0))
            self.lib["hsrmoveit"].set_base_vel(0.5)
            self.lib["hsrmoveit"].set_base_acc(0.5)

            ms_cfg = {"goal_pose": {"arm_lift_joint": 0.25,
                                    "arm_flex_joint": np.deg2rad(-150.0),
                                    "arm_roll_joint": np.deg2rad(180.0),
                                    "wrist_flex_joint": np.deg2rad(-60.0),
                                    "wrist_roll_joint": np.deg2rad(-90.0),
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": 0.0},
                    "exec_from_first": "all"}
            if i == 1:
                ms_cfg["goal_pose"]["arm_lift_joint"] = 0.5
            self.lib["hsrmoveit"].base_go_abs(GP_DRAWER_POSES[i],
                                              motion_synthesis_config=ms_cfg)
            rospy.sleep(1.0)

            # drawer handle TF
            handle_pose = self.lib["tf"].get_pose_with_offset(
                "odom",
                self.robot_descriptor["FRAME_MAP"],
                "handle",
                Pose(position=GP_HANDLE_POSES[i],
                     orientation=Quaternion(0, 0, 0, 1)))

            end_effector_q = self.lib["tf"].get_pose_with_offset(
                "odom", "hand_palm_link", "end_effector_pose",
                Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
            ).orientation
            end_effector_rpy = self.lib["tf"].quaternion2euler(end_effector_q)

            base_link_q = self.lib["tf"].get_pose_with_offset(
                "odom", "base_link", "base_link_pose",
                Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))
            ).orientation
            base_link_yaw = self.lib["tf"].quaternion2euler(base_link_q)[2]
            offset_x0, offset_y0 = self.lib["util"].rotate_coordinate(-0.10, 0, base_link_yaw)
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(-0.08, 0, base_link_yaw)

            self.lib["hsrmoveit"].set_base_vel(0.1)
            self.lib["hsrmoveit"].set_base_acc(0.1)
            self.lib["hsrmoveit"].set_whole_body_vel(0.5)
            self.lib["hsrmoveit"].set_whole_body_acc(0.5)

            pos = handle_pose.position
            self.lib["hsrmoveit"].whole_body_IK_cartesian_with_timeout(
                (pos.x + offset_x0, pos.x + offset_x1),
                (pos.y + offset_y0, pos.y + offset_y1),
                (pos.z, pos.z),
                (end_effector_rpy[0], end_effector_rpy[0]),
                (end_effector_rpy[1], end_effector_rpy[1]),
                (end_effector_rpy[2], end_effector_rpy[2]),
                eef_step=0.05,
                timeout=7.0)

            # grasp
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(-10.0), is_sync=False, time=3.0)

            # open
            offset_x0, offset_y0 = self.lib["util"].rotate_coordinate(GP_BACK_VAL[i]/2.0, 0.03, base_link_yaw)
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(GP_BACK_VAL[i], 0.02, base_link_yaw)
            self.lib["hsrmoveit"].whole_body_IK_cartesian_with_timeout(
                (pos.x + offset_x0, pos.x + offset_x1),
                (pos.y + offset_y0, pos.y + offset_y1),
                (pos.z, pos.z),
                (end_effector_rpy[0], end_effector_rpy[0]),
                (end_effector_rpy[1], end_effector_rpy[1]),
                (end_effector_rpy[2], end_effector_rpy[2]),
                eef_step=0.1,
                pos_tolerance=0.05,
                ori_tolerance=0.5,
                timeout=7.0)

            rospy.sleep(0.1)
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
            rospy.sleep(GP_GRIPPER_OPEN_TIME)

            # move to next
            currents = self.lib["hsrsub"].get_joint_positions()
            currents["arm_lift_joint"] = 0.6
            currents["arm_roll_joint"] = np.deg2rad(90.0)
            self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 1.0)
            if i == 0:
                rospy.sleep(1.5)
            else:
                rospy.sleep(0.5)
                # move to left
                sec = 1.0
                for i in xrange(int(sec/0.1)):
                    self.lib["hsrpub"].omni_base_vel(0, 0.2, 0)
                    rospy.sleep(0.1)

            self.lib["hsrmoveit"].set_base_vel(1.0)
            self.lib["hsrmoveit"].set_base_acc(1.0)
            self.lib["hsrmoveit"].set_whole_body_vel(1.0)
            self.lib["hsrmoveit"].set_whole_body_acc(1.0)

        return "next"


class Mapping(smach.State):
    """Mapping object state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        rospy.set_param(rospy.get_name() + "/mapping/place", "floor")
        return

    def __del__(self):
        return

    def add_objects(self, pan, tilt, num, search_place=""):
        self.lib["hsrpub"].whole_body_move_to_head_positions(
            {"head_pan_joint": pan, "head_tilt_joint": tilt}, 0.1)
        self.lib["hsrutil"].wait_goal_head_joints(
            {"head_pan_joint": pan, "head_tilt_joint": tilt}, timeout=1.0)
        rospy.sleep(1.0)

        ids = []
        names = []
        poses = []
        scores = []
        for i in range(num):
            yolact_with_pose = self.lib["yolact"].recognize().with_pose
            for j in range(yolact_with_pose.amount):
                if yolact_with_pose.is_valid_pose[j] is False:
                    continue

                ids.append(yolact_with_pose.id[j])
                names.append(GP_YCB_NAMES[yolact_with_pose.id[j]])
                pose_map = self.lib["tf"].get_pose_with_offset(
                    self.robot_descriptor["FRAME_MAP"],
                    self.robot_descriptor["FRAME_RGBD_POINT"],
                    "obj_%s_%s" % (i, j),
                    yolact_with_pose.pose[j])
                poses.append(pose_map)
                scores.append(yolact_with_pose.score[j])

        self.lib["action"].mapping_object(
            MappingAction.ADD,
            ids, names, poses, scores, search_place)
        
        return

    def execute(self, userdata):
        rospy.set_param(rospy.get_name() + "/deposit/drawer", False)

        # mapping floor
        place = rospy.get_param(rospy.get_name() + "/mapping/place", "floor")
        if "floor" in place:
            ms_cfg = {"goal_pose": {"arm_lift_joint": 0.0,
                                    "arm_flex_joint": 0.0,
                                    "arm_roll_joint": np.deg2rad(-90.0),
                                    "wrist_flex_joint": np.deg2rad(-110.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": np.deg2rad(-40.0),
                                    "head_tilt_joint": np.deg2rad(-50.0)},
                      "exec_from_first": ["arm_lift_joint",
                                          "arm_roll_joint",
                                          "wrist_flex_joint",
                                          "wrist_roll_joint"]}
            self.lib["hsrmoveit"].base_go_abs(
                GP_MAPPING_POSE[place][1], motion_synthesis_config=ms_cfg)

            num = 3
            min_pan = np.deg2rad(-40.0)
            max_pan = np.deg2rad(40.0)
            step = (max_pan - min_pan) / float(num - 1)
            for i in range(num):
                self.add_objects(min_pan + (step * i),
                                 np.deg2rad(-50.0), 5)

        # mapping table
        if "table" in place:
            ms_cfg = {"goal_pose": {"arm_lift_joint": 0.3,
                                    "arm_flex_joint": np.deg2rad(-130.0),
                                    "arm_roll_joint": np.deg2rad(0.0),
                                    "wrist_flex_joint": np.deg2rad(-110.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": np.deg2rad(40.0),
                                    "head_tilt_joint": np.deg2rad(4-0.0)},
                      "exec_from_first": "all"}

            self.lib["hsrmoveit"].base_go_abs(
                GP_MAPPING_POSE[place], motion_synthesis_config=ms_cfg)
            self.lib["hsraction"].gripper_apply_force(1.0, is_sync=False)

            num = 2
            min_pan = np.deg2rad(-15.0)
            max_pan = np.deg2rad(40.0)
            step = (max_pan - min_pan) / float(num - 1)
            for i in range(num):
                self.add_objects(max_pan - (step * i),
                                 np.deg2rad(-40.0), 5)

        # mapping
        self.lib["action"].mapping_object(MappingAction.MAPPING)
        rospy.set_param(rospy.get_name() + "/mapping/first", True)

        return "next"


class GoToRecPlace(smach.State):
    """Go to recognition place state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "under", "mapping", "loop", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        self.rec_pos = 0
        rospy.set_param(rospy.get_name() + "/search_place", "floor")
        rospy.set_param(rospy.get_name() + "/deposit/drawer", False)

        return

    def __del__(self):
        return

    def execute(self, userdata):
        x, y, yaw = self.lib["hsrutil"].get_robot_position_on_map()

        # move a little
        if rospy.get_param(rospy.get_name() + "/no_rec", False):
            x = np.random.rand() * 2.0
            y = np.random.rand() * 2.0
            rospy.set_param(rospy.get_name() + "/no_rec", False)

        # get nearest object
        search_place = rospy.get_param(rospy.get_name() + "/search_place", "floor")
        nearest_obj = self.lib["action"].mapping_object(
            MappingAction.GETNEAREST,
            search_place = search_place,
            pose = Pose(Point(x, y, 0), self.lib["tf"].euler2quaternion((0.0, 0.0, yaw))))

        # missing
        if not nearest_obj.result:
            if search_place == "floor":
                rospy.set_param(rospy.get_name() + "/search_place", "under_table")
                return "loop"
            elif search_place == "under_table":
                rospy.set_param(rospy.get_name() + "/search_place", "table")
                if rospy.get_param(rospy.get_name() + "/mapping/first", True):
                    rospy.set_param(rospy.get_name() + "/mapping/place", "table")
                return "mapping"
            else:
                rospy.set_param(rospy.get_name() + "/mapping/place", "floor")
                rospy.set_param(rospy.get_name() + "/search_place", "floor")
                return "mapping"

        # ROS param
        rospy.set_param(rospy.get_name() + "/mapping/first", False)
        rospy.set_param(rospy.get_name() + "/target_obj/id", nearest_obj.id[0])
        rospy.set_param(rospy.get_name() + "/target_obj/pose/x", nearest_obj.pose[0].position.x)
        rospy.set_param(rospy.get_name() + "/target_obj/pose/y", nearest_obj.pose[0].position.y)
        rospy.set_param(rospy.get_name() + "/target_obj/pose/z", nearest_obj.pose[0].position.z)
        rospy.set_param(rospy.get_name() + "/target_obj/score", nearest_obj.score[0])

        # After tiding the object into the drawer, the robot is rotated to avoid bumping.
        if rospy.get_param(rospy.get_name() + "/deposit/drawer", False):
            currents = self.lib["hsrsub"].get_joint_positions()
            currents["arm_flex_joint"] = np.deg2rad(0.1)
            self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 0.1)

            sec = 1.5
            for i in xrange(int(sec / 0.1)):
                self.lib["hsrpub"].omni_base_vel(0.0, 0, 1.5)
                rospy.sleep(0.1)

        # go to near the object
        self.lib["pub"].enable_obstacles(True)
        if "under_table" in nearest_obj.place[0] or "back" in nearest_obj.place[0] or nearest_obj.id[0] in ["19", "21", "24"]:
            ms_cfg = {"goal_pose": {"arm_lift_joint": 0.0,
                                    "arm_flex_joint": np.deg2rad(-135.0),
                                    "arm_roll_joint": np.deg2rad(0.0),
                                    "wrist_flex_joint": np.deg2rad(35.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": np.deg2rad(-30.0)},
                      "exec_from_first": "all"}

            p = nearest_obj.pose[0].position
            offset_x, offset_y = 0.05, -0.8
            rospy.set_param(rospy.get_name() + "/back/flag", False)
            if nearest_obj.place[0] == "under_table":
                stats = self.lib["hsrmoveit"].base_go_abs(
                    Pose2D(p.x + offset_x, p.y + offset_y, 1.57),
                    motion_synthesis_config=ms_cfg)
            else:
                if "back" in nearest_obj.place[0]:
                    rospy.set_param(rospy.get_name() + "/back/flag", True)
                
                ms_cfg["goal_pose"]["arm_lift_joint"] = p.z
                stats = self.lib["hsrmoveit"].base_go_abs(
                    Pose2D(p.x + offset_x, p.y + offset_y, 1.57),
                    motion_synthesis_config=ms_cfg)

            self.lib["pub"].enable_obstacles(False)
            return "under"

        elif "table" in nearest_obj.place[0]:
            ms_cfg = {"goal_pose": {"arm_lift_joint": nearest_obj.pose[0].position.z + 0.1,
                                    "arm_flex_joint": -0.1,
                                    "arm_roll_joint": np.deg2rad(0.0),
                                    "wrist_flex_joint": np.deg2rad(-90.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": np.deg2rad(-20.0)},
                      "exec_from_first": ["arm_lift_joint",
                                          "arm_roll_joint",
                                          "wrist_flex_joint",
                                          "wrist_roll_joint",
                                          "head_pan_joint",
                                          "head_tilt_joint"]}

            p = nearest_obj.pose[0].position
            offset_x, offset_y = 0.1, -0.5
            yaw = 1.57
            if p.x > 1.3 and p.y > 1.8:
                stats = self.lib["hsrmoveit"].base_go_abs(
                    Pose2D(p.x + 0.5, 1.2, yaw),
                    motion_synthesis_config=ms_cfg)

            ms_cfg["goal_pose"]["arm_flex_joint"] = np.deg2rad(-90.0)
            stats = self.lib["hsrmoveit"].base_go_abs(
                Pose2D(p.x + offset_x, p.y + offset_y, yaw),
                motion_synthesis_config=ms_cfg)

        else:                
            pose_base = self.lib["tf"].get_pose_with_offset(
                self.robot_descriptor["FRAME_BASE_FOOTPRINT"],
                self.robot_descriptor["FRAME_MAP"],
                "target_obj",
                Pose(Point(nearest_obj.pose[0].position.x, nearest_obj.pose[0].position.y, 0),
                     Quaternion(0, 0, 0, 1)))

            # radius r, inclination m
            r = 0.5
            xo = pose_base.position.x
            yo = pose_base.position.y
            m = yo / xo
            tmp = np.roots([1 + m*m, -2 * (xo + m * yo), xo*xo + yo*yo - r])
            ans_x = tmp[0] if abs(tmp[0]) < abs(tmp[1]) else tmp[1]
            ans_y = m * ans_x

            goal_place = self.lib["tf"].get_pose_with_offset(
                self.robot_descriptor["FRAME_MAP"],
                self.robot_descriptor["FRAME_BASE_FOOTPRINT"],
                "goal_place_tmp",
                Pose(Point(ans_x, ans_y, 0), Quaternion(0, 0, 0, 1)))
            theta = np.arctan2(nearest_obj.pose[0].position.y - y, nearest_obj.pose[0].position.x - x)

            ms_cfg = {"goal_pose": {"arm_lift_joint": nearest_obj.pose[0].position.z + 0.1,
                                    "arm_flex_joint": np.deg2rad(-90.0),
                                    "arm_roll_joint": np.deg2rad(0.0),
                                    "wrist_flex_joint": np.deg2rad(-90.0),
                                    "wrist_roll_joint": 0.0,
                                    "head_pan_joint": 0.0,
                                    "head_tilt_joint": np.deg2rad(-50.0)},
                      "exec_from_first": ["arm_lift_joint",
                                          "arm_roll_joint",
                                          "wrist_flex_joint",
                                          "wrist_roll_joint",
                                          "head_pan_joint",
                                          "head_tilt_joint"]}

            p = goal_place.position
            offset_x, offset_y = self.lib["util"].rotate_coordinate(0.2, -0.1, theta)
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
            stats = self.lib["hsrmoveit"].base_go_abs(
                Pose2D(p.x + offset_x, p.y + offset_y, theta),
                motion_synthesis_config=ms_cfg)

        self.lib["pub"].enable_obstacles(False)
        return "next"
        

class GetObject(smach.State):
    """Get object state."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "move", "loop", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        self.no_rec_cnt = 0
        return

    def __del__(self):
        return

    def grasp_pose_estimation(self, ):
        self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
        rospy.sleep(1.0)

        smi_hand = self.lib["hsrsub"].get_hand_image(is_cv=False)
        rec_res = self.lib["yolact"].recognize(rgb=smi_hand)
        
        cv_rgb = self.lib["opencv"].smi2cv(rec_res.without_pose.rgb, "bgr8")
        grasp_center = (265, int(cv_rgb.shape[0]/2.0))
        # cv2.circle(cv_rgb, grasp_center, 5, (0, 0, 255), -1)

        min_distance = np.inf
        distances = [-1, -1]
        index = -1
        for i in range(rec_res.without_pose.amount):
            cx, cy = int(rec_res.without_pose.x[i] + rec_res.without_pose.w[i]/2.0), int(rec_res.without_pose.y[i] + rec_res.without_pose.h[i]/2.0)
            distance_x, distance_y = grasp_center[0] - cx, grasp_center[1] - cy
            distance = math.sqrt(pow(distance_x, 2) + pow(distance_y, 2))
            if distance > min_distance:
                continue
            min_distance = distance
            distances = [distance_x, distance_y]
            index = i

        cv_mask = self.lib["opencv"].smi2cv(rec_res.without_pose.mask, "mono8")
        cv_mask_obj = np.zeros(cv_mask.shape, np.uint8)
        cv_mask_obj[:, :] = np.where(cv_mask == index+1, 255, 0)
        cv_mask_obj_rot = cv2.rotate(cv_mask_obj, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # bounding rect
        try:
            _, contours, _ = cv2.findContours(cv_mask_obj_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        except:
            contours, _ = cv2.findContours(cv_mask_obj_rot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours == []:
            pass # TODO
        max_contour = max(contours, key=lambda x: cv2.contourArea(x))
        rect = cv2.minAreaRect(max_contour)

        # calc angle
        box = np.int0(cv2.boxPoints(rect))
        vec1, vec2 = box[0] - box[1], box[1] - box[2]
        vecw = vec1 if np.linalg.norm(vec1) < np.linalg.norm(vec2) else vec2 # short vector
        angle = np.arctan(vecw[1] / float(vecw[0])) if not np.isclose(vecw[0], 0) else -1.57
        if np.rad2deg(angle) > -92.0 and np.rad2deg(angle) < -88.0:
            angle += 3.14
        x = -distances[0] / 1800.0
        y = -distances[1] / 1300.0

        return x, y, angle

    def execute(self, userdata):
        offset_x, offset_y, angle = self.grasp_pose_estimation()

        # add obstacles from world model
        # self.lib["hsrmoveit"].delete()
        # for key in GP_COLLISION_MODELS.keys():
        #     self.lib["hsrmoveit"].add_model_from_sdf(GP_COLLISION_MODELS[key][0], GP_COLLISION_MODELS[key][1],
        #         roslib.packages.get_pkg_dir("hma_hsr_sim_pkg") + "/io/models/" + str(key) + "/model.sdf")
        # all_obj = self.lib["action"].mapping_object(MappingAction.GETALL)
        # for i in range(all_obj.amount):
        #     self.lib["hsrmoveit"].add_box("obj_" + str(i), "map", all_obj.pose[i].position, (0.05, 0.05, 0.05))
        
        grasp_obj_id = rospy.get_param(rospy.get_name() + "/target_obj/id", "0")
        if grasp_obj_id == "24": # bowl
            offset_y = 0.15
        elif grasp_obj_id == "29": # plate
            offset_y = 0.1
        else:
            offset_z = rospy.get_param(rospy.get_name() + "/target_obj/pose/z", 0.0) + 0.02

        hand_pos = self.lib["tf"].get_pose_with_offset(
                "odom", "hand_palm_link", "hand_pose",
                Pose(position=Point(offset_x, offset_y, 0),
                     orientation=self.lib["tf"].euler2quaternion((0, 0, angle-3.14))))

        stats = False
        pos = hand_pos.position
        cnt = 0
        while stats is False:
            if cnt >= 10:
                rospy.set_param(rospy.get_name() + "/no_rec", True)
                x = rospy.get_param(rospy.get_name() + "/target_obj/pose/x", 0.0)
                y = rospy.get_param(rospy.get_name() + "/target_obj/pose/y", 0.0)
                z = rospy.get_param(rospy.get_name() + "/target_obj/pose/z", 0.0)
                self.lib["action"].mapping_object(
                    MappingAction.DELETE,
                    id = grasp_obj_id,
                    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                    score = rospy.get_param(rospy.get_name() + "/target_obj/score", 0.0)
                )
                return "move"

            if grasp_obj_id in GP_NON_DIRECTION_OBJ_IDS:
                hand_pos = self.lib["tf"].get_pose_with_offset(
                        "odom", "hand_palm_link", "hand_pose_non_direction",
                        Pose(position=Point(offset_x, offset_y, 0),
                             orientation=self.lib["tf"].euler2quaternion((0, 0, -3.14))))
                offset_x += 0.01
            offset_theta = self.lib["tf"].quaternion2euler(hand_pos.orientation)[2]

            if cnt == 0:
                stats = self.lib["hsrmoveit"].whole_body_IK_cartesian(
                    [pos.x, pos.x],
                    [pos.y, pos.y],
                    [pos.z, offset_z],
                    [0.0, 0.0],
                    [np.deg2rad(180.0), np.deg2rad(180.0)],
                    [offset_theta, offset_theta],
                    eef_step = 0.1)
            else:
                stats = self.lib["hsrmoveit"].whole_body_IK_cartesian(
                    [pos.x],
                    [pos.y],
                    [offset_z + cnt*0.02],
                    [0.0],
                    [np.deg2rad(180.0)],
                    [offset_theta],
                    eef_step = 0.1,
                    pos_tolerance=0.05,
                    ori_tolerance=0.5)
            cnt += 1

        rospy.sleep(0.1)

        orient_item = False
        if grasp_obj_id in ["30", "31", "32", "40"]:
            orient_item = True

        pose_map = self.lib["tf"].get_pose_with_offset(
            "map", "hand_palm_link", "hand_pose_map",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1)))

        # grasp
        if grasp_obj_id in ["19", "53"]: # pitcher_base, soccer_ball
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(30.0), is_sync=False, time=2.0)
        elif orient_item:
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(-40.0), is_sync=False, time=4.0)
            rospy.set_param(rospy.get_name() + "/oreint_item/flag", True)
        else:
            self.lib["hsraction"].gripper_apply_force(1.0)
            rospy.sleep(0.5)
            currents = self.lib["hsrsub"].get_joint_positions()
            if currents["hand_motor_joint"] < 0.1:
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(-20.0), is_sync=False, time=2.0)

        # raise the arm
        currents = self.lib["hsrsub"].get_joint_positions()
        currents["arm_lift_joint"] += 0.05
        self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 0.1)
        rospy.sleep(0.1)

        # backward
        sec = 1.5
        for i in xrange(int(sec / 0.1)):
            self.lib["hsrpub"].omni_base_vel(-0.2, 0, 0)
            rospy.sleep(0.1)

        if orient_item:
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(-40.0), is_sync=False, time=1.0)

        # delete obstacles
        self.lib["hsrmoveit"].delete()

        # delete object from mapping data
        self.lib["action"].mapping_object(
            MappingAction.DELETE, id=grasp_obj_id, pose=pose_map, score=1.0)

        # ROS param
        rospy.set_param(rospy.get_name() + "/obj_id", grasp_obj_id)

        return "next"


class GetObjectFromUnder(smach.State):
    """Get object from under the table."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "move", "loop", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        return

    def __del__(self):
        return

    def execute(self, userdata):
        self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
        rospy.sleep(1.0)

        end_effector_q = self.lib["tf"].get_pose_with_offset(
            "odom", "hand_palm_link", "end_effector_pose",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
        end_effector_rpy = self.lib["tf"].quaternion2euler(end_effector_q)

        base_link_q = self.lib["tf"].get_pose_with_offset(
            "odom", "base_link", "base_link_pose",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
        base_link_yaw = self.lib["tf"].quaternion2euler(base_link_q)[2]

        x = rospy.get_param(rospy.get_name() + "/target_obj/pose/x", 0.0)
        y = rospy.get_param(rospy.get_name() + "/target_obj/pose/y", 0.0)
        z = rospy.get_param(rospy.get_name() + "/target_obj/pose/z", 0.0)
        pos = self.lib["tf"].get_pose_with_offset(
            "odom", "map", "grasp_obj",
            Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 1))).position

        grasp_obj_id = rospy.get_param(rospy.get_name() + "/target_obj/id", "0")

        stats = -1
        offset_x, offset_y = -0.2, 0
        offset_x0, offset_y0 = self.lib["util"].rotate_coordinate(offset_x, offset_y, base_link_yaw)
        offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(0.0, 0, base_link_yaw)
        
        if rospy.get_param(rospy.get_name() + "/back/flag", False):
            offset_z = z
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(-0.03, 0, base_link_yaw)
        elif grasp_obj_id == "19":
            offset_z = z - 0.05
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(0.0, 0, base_link_yaw)
        elif grasp_obj_id == "21":
            offset_z = z - 0.03
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(-0.02, 0, base_link_yaw)
        elif grasp_obj_id == "24":
            offset_z = z
            offset_x1, offset_y1 = self.lib["util"].rotate_coordinate(-0.05, 0, base_link_yaw)
        else:
            offset_z = 0.03


        stats = self.lib["hsrmoveit"].whole_body_IK_cartesian_with_timeout(
            [pos.x + offset_x0, pos.x + offset_x1],
            [pos.y + offset_y0, pos.y + offset_y1],
            [offset_z, offset_z],
            [end_effector_rpy[0], end_effector_rpy[0]],
            [end_effector_rpy[1], end_effector_rpy[1]],
            [end_effector_rpy[2], end_effector_rpy[2]],
            eef_step = 0.05,
            timeout=5.0)
        if stats is False:
            rospy.set_param(rospy.get_name() + "/no_rec", True)
            return "move"
        rospy.sleep(0.1)

        orient_item = False
        if grasp_obj_id in ["30", "31", "32", "40"]:
            orient_item = True

        pose_map = self.lib["tf"].get_pose_with_offset(
            "map", "hand_palm_link", "hand_pose_map",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1)))


        # grasp
        if grasp_obj_id in ["19", "53"]: # pitcher_base, soccer_ball
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(30.0), is_sync=False, time=2.0)
        elif orient_item:
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(-40.0), is_sync=False, time=3.0)
        else:
            self.lib["hsraction"].gripper_apply_force(1.0)
            rospy.sleep(0.5)
            currents = self.lib["hsrsub"].get_joint_positions()
            if currents["hand_motor_joint"] < 0.1:
                self.lib["hsrmoveit"].gripper_command(np.deg2rad(-20.0), is_sync=False, time=2.0)

        # raise the arm
        currents = self.lib["hsrsub"].get_joint_positions()
        currents["arm_lift_joint"] += 0.03
        self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 0.1)
        rospy.sleep(0.1)

        # backward
        sec = 2.0
        for i in xrange(int(sec / 0.1)):
            self.lib["hsrpub"].omni_base_vel(-0.2, 0, 0)
            rospy.sleep(0.1)

        if orient_item:
            self.lib["hsrmoveit"].gripper_command(np.deg2rad(-40.0), is_sync=False, time=1.0)

        # delete obstacles
        self.lib["hsrmoveit"].delete()

        # delete object from mapping data
        self.lib["action"].mapping_object(MappingAction.DELETE,
                                          id=grasp_obj_id,
                                          pose=pose_map,
                                          score=1.0)

        # ROS param
        rospy.set_param(rospy.get_name() + "/obj_id", grasp_obj_id)

        return "next"


class PlaceObject(smach.State):
    """Place object to deposit place."""

    def __init__(self, lib={}, robot_descriptor={}):
        smach.State.__init__(self, outcomes = ["next", "except"])

        self.lineno = inspect.currentframe().f_lineno
        self.lib = lib
        self.robot_descriptor = robot_descriptor

        self.food_pos = 0
        return

    def __del__(self):
        return

    def place_orient_item(self):
        rospy.sleep(1.0)

        end_effector_q = self.lib["tf"].get_pose_with_offset(
            "odom", "hand_palm_link", "end_effector_pose",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
        end_effector_rpy = self.lib["tf"].quaternion2euler(end_effector_q)

        base_link_q = self.lib["tf"].get_pose_with_offset(
            "odom", "base_link", "base_link_pose",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).orientation
        base_link_yaw = self.lib["tf"].quaternion2euler(base_link_q)[2]

        pos = self.lib["tf"].get_pose_with_offset(
            "odom", "container_a", "hand_pose",
            Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))).position

        stats = -1
        offset_x, offset_y = -0.1, -0.03
        offset_x0, offset_y0 = self.lib["util"].rotate_coordinate(offset_x, offset_y, base_link_yaw)
        cnt = 0

        self.lib["hsrmoveit"].whole_body_IK_cartesian_with_timeout(
            [pos.x + offset_x0, pos.x + offset_x0],
            [pos.y + offset_y0, pos.y + offset_y0],
            [0.75, 0.6],
            [end_effector_rpy[0], end_effector_rpy[0]],
            [end_effector_rpy[1], end_effector_rpy[1]],
            [end_effector_rpy[2], end_effector_rpy[2]],
            eef_step = 0.05,
            timeout=5.0)

        rospy.sleep(0.1)
        return

    def execute(self, userdata):
        rospy.set_param(rospy.get_name() + "/deposit/drawer", True) # FIXME

        # orientation items
        ms_cfg = {"goal_pose": {"arm_lift_joint": 0.15,
                                "arm_flex_joint": np.deg2rad(-15.0),
                                "arm_roll_joint": 0.0,
                                "wrist_flex_joint": np.deg2rad(-75.0),
                                "wrist_roll_joint": 0.0 if rospy.get_param(rospy.get_name() + "/oreint_item/direction", "left") == "left" else np.deg2rad(178.0),
                                "head_pan_joint": 0.0,
                                "head_tilt_joint": 0.0}}

        if rospy.get_param(rospy.get_name() + "/oreint_item/flag", False):
            stats = self.lib["hsrmoveit"].base_go_abs(GP_DEPOSIT_POSES["orientation"],
                                                      motion_synthesis_config=ms_cfg)

            self.place_orient_item()
            rospy.set_param(rospy.get_name() + "/oreint_item/flag", False)

            self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
            rospy.sleep(GP_GRIPPER_OPEN_TIME)

            currents = self.lib["hsrsub"].get_joint_positions()
            currents["arm_lift_joint"] += 0.15
            self.lib["hsrpub"].whole_body_move_to_arm_positions(currents, 1.0)
            rospy.sleep(1.0)

            return "next"

        # others
        ms_cfg = {"goal_pose": {"arm_lift_joint": 0.6,
                                "arm_flex_joint": np.deg2rad(-90.0),
                                "arm_roll_joint": 0.0,
                                "wrist_flex_joint": np.deg2rad(-90.0),
                                "wrist_roll_joint": 0.0,
                                "head_pan_joint": 0.0,
                                "head_tilt_joint": 0.0},
                    "exec_from_first": ["arm_lift_joint",
                                        "arm_roll_joint",
                                        "wrist_flex_joint",
                                        "wrist_roll_joint"]}

        obj_id = int(rospy.get_param(rospy.get_name() + "/obj_id", "0"))

        # self.lib["pub"].enable_obstacles(True)
        flag = False
        # tools
        if obj_id >= GP_CATEGORY_ID_RANGES["tool"]["min"] and obj_id <= GP_CATEGORY_ID_RANGES["tool"]["max"]:
            rospy.set_param(rospy.get_name() + "/deposit/drawer", True)
            stats = self.lib["hsrmoveit"].base_go_abs(
                GP_DEPOSIT_POSES["tool"], motion_synthesis_config=ms_cfg)

            joints = {"arm_lift_joint": 0.45}
            self.lib["hsrpub"].whole_body_move_to_arm_positions(joints, 1.0)

        # shape items
        elif obj_id >= GP_CATEGORY_ID_RANGES["shape"]["min"] and obj_id <= GP_CATEGORY_ID_RANGES["shape"]["max"]:
            rospy.set_param(rospy.get_name() + "/deposit/drawer", True)
            stats = self.lib["hsrmoveit"].base_go_abs(
                GP_DEPOSIT_POSES["shape"], motion_synthesis_config=ms_cfg)
            flag = True

            joints = {"arm_lift_joint": 0.2}
            self.lib["hsrpub"].whole_body_move_to_arm_positions(joints, 1.0)
            rospy.sleep(1.0)

        # kitchen items            
        elif obj_id >= GP_CATEGORY_ID_RANGES["kitchen"]["min"] and obj_id <= GP_CATEGORY_ID_RANGES["kitchen"]["max"]:
            if obj_id in [19, 29]: # pitcher base, plate
                stats = self.lib["hsrmoveit"].base_go_abs(
                    GP_DEPOSIT_POSES["task"], motion_synthesis_config=ms_cfg)
            else:
                x, _, _ = self.lib["hsrutil"].get_robot_position_on_map()
                if x < GP_DEPOSIT_POSES["kitchen"][0].x: # avoid bumping into drawer
                    _ms_cfg = copy.deepcopy(ms_cfg)
                    _ms_cfg["goal_pose"]["arm_flex_joint"] = 0.0
                    _ms_cfg["goal_pose"]["wrist_roll_joint"] = np.deg2rad(-90.0)
                    stats = self.lib["hsrmoveit"].base_go_abs(
                        GP_DEPOSIT_POSES["kitchen"][0], motion_synthesis_config=_ms_cfg)

                    _ms_cfg["goal_pose"]["arm_flex_joint"] = np.deg2rad(-90.0)
                    _ms_cfg["exec_from_fisrt"] = "all"
                    stats = self.lib["hsrmoveit"].base_go_abs(
                        GP_DEPOSIT_POSES["kitchen"][1], motion_synthesis_config=_ms_cfg)
                else:
                    _ms_cfg = copy.deepcopy(ms_cfg)
                    _ms_cfg["goal_pose"]["wrist_roll_joint"] = np.deg2rad(-90.0)
                    stats = self.lib["hsrmoveit"].base_go_abs(
                        Pose2D(GP_DEPOSIT_POSES["kitchen"][1].x,
                               GP_DEPOSIT_POSES["kitchen"][1].y + 0.05,
                               GP_DEPOSIT_POSES["kitchen"][1].theta),
                        motion_synthesis_config=_ms_cfg)

                joints = {"arm_lift_joint": 0.45}
                self.lib["hsrpub"].whole_body_move_to_arm_positions(joints, 1.0)

            flag = True

        # food
        elif obj_id >= GP_CATEGORY_ID_RANGES["food"]["min"] and obj_id <= GP_CATEGORY_ID_RANGES["food"]["max"]:
            _ms_cfg = copy.deepcopy(ms_cfg)
            _ms_cfg["goal_pose"]["arm_lift_joint"] = 0.4
            _ms_cfg["goal_pose"]["wrist_flex_joint"] = np.deg2rad(-45.0)

            stats = self.lib["hsrmoveit"].base_go_abs(
                GP_DEPOSIT_POSES["food"][self.food_pos % 2], motion_synthesis_config=_ms_cfg)
            self.food_pos += 1

        # task items
        elif obj_id >= GP_CATEGORY_ID_RANGES["task"]["min"] and obj_id <= GP_CATEGORY_ID_RANGES["task"]["max"]:
            _ms_cfg = copy.deepcopy(ms_cfg)
            _ms_cfg["goal_pose"]["arm_lift_joint"] = 0.3
            stats = self.lib["hsrmoveit"].base_go_abs(
                GP_DEPOSIT_POSES["task"], motion_synthesis_config=_ms_cfg)
                
        else:
            _ms_cfg = copy.deepcopy(ms_cfg)
            _ms_cfg["goal_pose"]["arm_lift_joint"] = 0.3
            stats = self.lib["hsrmoveit"].base_go_abs(
                GP_DEPOSIT_POSES["task"], motion_synthesis_config=_ms_cfg)

        # self.lib["pub"].enable_obstacles(False)

        self.lib["hsrmoveit"].gripper_command(np.deg2rad(60.0))
        rospy.sleep(GP_GRIPPER_OPEN_TIME)
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
