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

import warnings
warnings.simplefilter('ignore')

import sys
import roslib
import moveit_commander
import xml.etree.ElementTree as ET

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_msgs.msg import PlanningScene, ObjectColor

from hma_hsr_nav_action.msg import MotionSynthesisAction
from hma_hsr_nav_action.msg import MotionSynthesisGoal
from hma_hsr_msgs.msg import HSRJoints


class LibHSRMoveit:
    """HSR MoveIt library for WRS simlator."""

    def __init__(self, libtf, libhsrutil, libhsrpub):
        """Constractor.

        Args:
            libtf (instance): TF library instance
            libhsrpub (instance): HSR Pub library instance
        """
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.lib = {"tf": libtf, "hsrutil": libhsrutil, "hsrpub": libhsrpub}

        # Moveit commander
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        self.base = moveit_commander.MoveGroupCommander("base")
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.head = moveit_commander.MoveGroupCommander("head")
        self.scene = moveit_commander.PlanningSceneInterface()

        # Moveit settings
        self.whole_body.allow_replanning(True)
        self.whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])
        self.whole_body.set_num_planning_attempts(10)
        self.whole_body.set_planning_time(5.0)
        # self.whole_body.set_goal_position_tolerance(0.01) # m
        # self.whole_body.set_goal_orientation_tolerance(0.1) # deg
        # self.whole_body.set_goal_joint_tolerance(0.1)
        self.whole_body.set_max_acceleration_scaling_factor(1.0)
        self.whole_body.set_max_velocity_scaling_factor(1.0)

        self.base.allow_replanning(True)
        self.base.set_workspace([-10.0, -10.0, 10.0, 10.0])
        self.base.set_max_acceleration_scaling_factor(1.0)
        self.base.set_max_velocity_scaling_factor(1.0)
        self.base.set_num_planning_attempts(10)
        self.base.set_planning_time(5.0)

        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.gripper.set_max_acceleration_scaling_factor(1.0)
        self.gripper.set_max_velocity_scaling_factor(1.0)
        self.head.set_max_acceleration_scaling_factor(1.0)
        self.head.set_max_velocity_scaling_factor(1.0)

        # ROS I/F
        self.ac_motion_synthesis = actionlib.SimpleActionClient(
            "/motion_synthesis", MotionSynthesisAction)
        # self.ac_motion_synthesis.wait_for_server()

        # Initialize
        self.tf_index = 0    

        return

    def delete(self):
        self.scene.remove_world_object()
        for key in self.lib.keys():
            self.lib[key].delete()
        return
    
    def set_base_vel(self, vel=1.0):
        """Set the maximum speed of the base.

        Args:
            vel (float, optional): Maximum speed value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """
        self.base.set_max_velocity_scaling_factor(float(vel))
        return

    def set_base_acc(self, acc=1.0):
        """Set the maximum acceleration of the base.

        Args:
            vel (float, optional): Maximum acceleration value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """
        self.base.set_max_acceleration_scaling_factor(float(acc))
        return

    def set_whole_body_vel(self, vel=1.0):
        """Set the maximum speed of the whole_body.

        Args:
            vel (float, optional): Maximum speed value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """
        self.whole_body.set_max_velocity_scaling_factor(float(vel))
        return

    def set_whole_body_acc(self, acc=1.0):
        """Set the maximum acceleration of the whole_body.

        Args:
            vel (float, optional): Maximum acceleration value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """
        self.whole_body.set_max_acceleration_scaling_factor(float(acc))
        return

    def all_close(self, goal, actual, pos_tolerance, ori_tolerance):
        """Judges whether the actual value is within the tolerance for the goal value.

        Args:
            goal (list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped): The goal value.
            actual (list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped): The actual value.
            pos_tolerance (float): The tolerance of positions.
            ori_tolerance (float): The tolerance of orientations.

        Returns:
            bool: Return True if the value is within the tolerance.
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if index > 2:
                    if abs(abs(actual[index]) - abs(goal[index])) > ori_tolerance:
                        return False
                else:
                    if abs(actual[index] - goal[index]) > pos_tolerance:
                        return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual, pos_tolerance, ori_tolerance)

        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), pos_tolerance, ori_tolerance)

        return True

    def add_box(self, name, frame, pose, size):
        """Add a box to the planning scene.

        Args:
            name (str): The name of box.
            frame (str): The name of frame.
            pose (geometry_msgs/Point, geometry_msgs/Pose): Coordinates to add a box.
            size (list[float]): The size of box. The size is given as a (x, y, z).
        
        Examples:
            add_box("box_0", "world", Point(0.0, 0.1, 0.2), (0.1, 0.1, 0.1))
        """
        box_pose = PoseStamped()
        box_pose.header.frame_id = frame
        if isinstance(pose, Point):
            box_pose.pose = Pose(position=pose, orientation=Quaternion(0, 0, 0, 1))
        elif isinstance(pose, Pose):
            box_pose.pose = pose
        self.scene.add_box(name, box_pose, size=size)

        return

    def add_cylinder(self, name, frame, pose, height, radius):
        """Add a cylinder to the planning scene.

        Args:
            name (str): The name of box.
            frame (str): The name of frame.
            pose (geometry_msgs/Point, geometry_msgs/Pose): Coordinates to add a cylinder.
            height (float): The height of cylinder.
            radius (float): The radius of cylinder.
        
        Examples:
            add_cylinder("cylinder_0", "world", Point(0.0, 0.1, 0.2), (0.1, 0.1, 0.1))
        """
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = frame
        if isinstance(pose, Point):
            cylinder_pose.pose = Pose(position=pose, orientation=Quaternion(0, 0, 0, 1))
        elif isinstance(pose, Pose):
            cylinder_pose.pose = pose
        self.scene.add_cylinder(name, cylinder_pose, height, radius)


        return 

    def add_model_from_sdf(self, frame, point, euler, sdf):
        """Add a model from sdf to the planning scene.

        Args:
            frame (str): The name of frame.
            point (geometry_msgs/Point): Coordinates to add a model.
            euler (geometry_msgs/Point): Euler angles of the model to be added.
            sdf (str): SDF file path.
        
        Examples:
            add_model_from_sdf("world", Point(0.0, 0.1, 0.2), Point(0.0, 0.0, 1.57))
        """
        tree = ET.parse(sdf)
        root = tree.getroot()
        frame_rot_cnt = 0
        for model in root:
            model_name = model.attrib["name"]
            for child in model:
                if child.tag != "link":
                    continue

                for collision in child:
                    if collision.tag != "collision":
                        continue

                    parts_name = collision.attrib["name"]
                    parts_pose = collision.find("pose").text.split(" ")

                    for geometry in collision:
                        if geometry.tag != "geometry":
                            continue
                        
                        for form in geometry:
                            q = self.lib["tf"].euler2quaternion((
                                float(parts_pose[3]),
                                float(parts_pose[4]),
                                float(parts_pose[5])))

                            if euler != Point(0, 0, 0):
                                self.lib["tf"].send_transform(
                                    frame + "_rot_" + str(frame_rot_cnt),
                                    frame,
                                    Pose(Point(0, 0, 0),
                                         self.lib["tf"].euler2quaternion(euler)))
                                frame = frame + "_rot_" + str(frame_rot_cnt)
                                frame_rot_cnt += 1
                                rospy.sleep(0.1)
                            
                            if form.tag == "box":
                                parts_size = form.find("size").text.split(" ")

                                self.addBox(
                                    model_name + "_" + parts_name,
                                    frame,
                                    Pose(
                                        Point(
                                            point.x + float(parts_pose[0]),
                                            point.y + float(parts_pose[1]),
                                            point.z + float(parts_pose[2])),
                                        q),
                                    (float(parts_size[0]), float(parts_size[1]), float(parts_size[2])))

                            elif form.tag == "cylinder":
                                parts_height = float(form.find("length").text)
                                parts_radius = float(form.find("radius").text)

                                self.addCylinder(
                                    model_name + "_" + parts_name,
                                    frame,
                                    Pose(
                                        Point(
                                            point.x + float(parts_pose[0]),
                                            point.y + float(parts_pose[1]),
                                            point.z + float(parts_pose[2])),
                                        q),
                                    parts_height,
                                    parts_radius)

        return

    def whole_body_IK(self, x, y, z, roll, pitch, yaw, frame="odom", pos_tolerance=0.02, ori_tolerance=0.17):
        """Inverse kinematics control of the whole body.

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            frame (str, optional): The name of reference frame. Defaults to "odom".
            pos_tolerance (float): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float): The tolerance of orientations. Defaults to 0.17[rad].

        Returns:
            bool: Execution result.
        """
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.whole_body.set_pose_target(p)
        self.whole_body.go(wait=True)
        self.whole_body.stop()
        self.whole_body.clear_pose_targets()
        self.whole_body.clear_path_constraints()

        current_pose = self.whole_body.get_current_pose().pose
        return self.all_close(p, current_pose, pos_tolerance, ori_tolerance)

    def whole_body_IK_cartesian(self, x, y, z, roll, pitch, yaw, 
                                is_avoid_obstacle=True, 
                                eef_step=0.01,
                                fraction_th=0.5,
                                pos_tolerance=0.02, 
                                ori_tolerance=0.17):
        """Inverse kinematics control of the whole body for multiple coordinates.

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            is_avoid_obstacle (str, optional): Whether or not to avoid obstacles using planning scene.
                Defaults to True.
            eef_step (float, optional): Operating calculation steps. Defaults to 0.01[m].
            fraction_th (float, optional): This function is not executed if the calculated fraction is below the threshold.
                If the calculation was incomplete, fraction indicates the percentage of paths calculated (number of waypoints passed).
                Defaults to 0.5[m].
            pos_tolerance (float): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float): The tolerance of orientations. Defaults to 0.17[rad].

        Returns:
            bool: Execution result.
        """
        p = Pose()
        waypoints = []
        for i in range(len(x)):
            p.position.x = x[i]
            p.position.y = y[i]
            p.position.z = z[i]
            q = quaternion_from_euler(roll[i], pitch[i], yaw[i])
            p.orientation = Quaternion(q[0], q[1], q[2], q[3])

            waypoints.append(copy.deepcopy(p))

        plan, fraction = self.whole_body.compute_cartesian_path(
            waypoints, eef_step, 0.0, is_avoid_obstacle)
        if fraction < fraction_th:
            self.whole_body.stop()
            self.whole_body.clear_pose_targets()
            self.whole_body.clear_path_constraints()
            return False

        self.whole_body.execute(plan)
        self.whole_body.stop()
        self.whole_body.clear_pose_targets()
        self.whole_body.clear_path_constraints()

        current_pose = self.whole_body.get_current_pose().pose
        if self.all_close(p, current_pose, pos_tolerance, ori_tolerance):
            return True
        else:
            return False

    def whole_body_IK_cartesian_with_timeout(self, x, y, z, roll, pitch, yaw, 
                                             is_avoid_obstacle=True, 
                                             eef_step=0.01,
                                             fraction_th=0.5,
                                             pos_tolerance=0.02, 
                                             ori_tolerance=0.17,
                                             timeout=5.0):
        """Inverse kinematics control of the whole body for multiple coordinates with timeout.

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            is_avoid_obstacle (str, optional): Whether or not to avoid obstacles using planning scene.
                Defaults to True.
            eef_step (float, optional): Operating calculation steps. Defaults to 0.01[m].
            fraction_th (float, optional): This function is not executed if the calculated fraction is below the threshold.
                If the calculation was incomplete, fraction indicates the percentage of paths calculated (number of waypoints passed).
                Defaults to 0.5[m].
            pos_tolerance (float, optional): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float, optional): The tolerance of orientations. Defaults to 0.17[rad].
            timeout (float, optional): Timeout time. Defaults to 5.0[sec].

        Returns:
            bool: Execution result.
        """
        stats = False
        start = rospy.Time.now()
        while stats is False:
            if rospy.Time.now() - start > rospy.Duration(timeout):
                rospy.logwarn("[" + rospy.get_name() + "]: whole_body_IK_cartesian is terminated by timeout.")
                return False

            stats = self.whole_body_IK_cartesian(x, y, z, roll, pitch, yaw,
                                                 is_avoid_obstacle, eef_step, fraction_th,
                                                 pos_tolerance, ori_tolerance)
        return True

    def base_go_abs(self, pose, frame="map", motion_synthesis_config={}):
        """Move in absolute coordinates. It loops infinitely until the goal is reached.

        Args:
            pose (geometry_msgs/Pose2D, list[geometry_msgs/Pose2D]): Goal place positions.
                Multiple positions can also be specified in a list. 
                In such a case, the robot is moved in order.
            frame (str): The name of reference frame.
            motion_synthesis_config (dict, optional): If you want to apply the motion synthesis, set the configuration.
                Defaults to {} (Disable).
                The keys for config are as follows,
                    start_pose: Transition posture at start.
                        If "auto" is specified, it is automatically set to the go posture of HSR.
                    goal_pose: Transition posture at goal.
                    exec_from_first: Joint name list to transition from the first.
                        If "all" is specified, all specified joints execute from first.
                    goal_tolerance: Timing of starting motion synthesis (distance from the goal).
        Examples:
            base_go_abs(Pose2D(0.0, 1.0, 1.57), 
                        motion_synthesis_config={"start_pose":"auto", 
                                                 "goal_pose":{"arm_lift_joint": 0.1, ...}, 
                                                 "exec_from_first": ["arm_lift_joint"]},
                                                 "goal_tolerance": Point(0.3, 0.3, 0.0))
        """
        def exec_motion_synthesis(pose, config):
            goal = MotionSynthesisGoal()
            goal.goal_place = pose
            goal.apply_start_pose = False
            if "start_pose" in config.keys():
                goal.apply_start_pose = True
                if config["start_pose"] == "auto":
                    start_pose = {"arm_lift_joint": 0.0,
                                  "arm_flex_joint": 0.0,
                                  "arm_roll_joint": np.deg2rad(-90.0),
                                  "wrist_flex_joint": np.deg2rad(-90.0),
                                  "wrist_roll_joint": 0.0,
                                  "head_pan_joint": 0.0,
                                  "head_tilt_joint": np.deg2rad(-30.0)}
                    goal.start_pose = self.lib["hsrutil"].dict_to_hsr_joints(start_pose)
                else:
                    goal.start_pose = self.lib["hsrutil"].dict_to_hsr_joints(config["start_pose"])
            goal.apply_goal_pose = False
            if "goal_pose" in config.keys():
                goal.apply_goal_pose = True
                goal.goal_pose = self.lib["hsrutil"].dict_to_hsr_joints(config["goal_pose"])
                if "exec_from_first" in config.keys():
                    if config["exec_from_first"] == "all":
                        goal.exec_from_first = config["goal_pose"].keys()
                    else:
                        goal.exec_from_first = config["exec_from_first"]
                if "goal_tolerance" in config.keys():
                    goal.goal_tolerance = config["goal_tolerance"]
                else:
                    goal.goal_tolerance = Point(0.3, 0.3, 0.0)
            self.ac_motion_synthesis.send_goal(goal)

        rospy.loginfo("[" + rospy.get_name() + "]: NavState.START")
        if isinstance(pose, list):
            goal_places = []
            for i in range(len(pose)):
                goal_places.append(
                    self.lib["tf"].get_pose_with_offset(
                        self.base.get_planning_frame(),
                        frame,
                        "goal_place_" + str(i),
                        Pose(position=Point(pose[i].x, pose[i].y, 0.0),
                             orientation=self.lib["tf"].euler2quaternion((0.0, 0.0, pose[i].theta)))))

            exec_motion_synthesis(pose[-1], motion_synthesis_config)

            for i in range(len(pose)):
                self.base.set_joint_value_target("odom_x", goal_places[i].position.x)
                self.base.set_joint_value_target("odom_y", goal_places[i].position.y)
                _, _, yaw = self.lib["tf"].quaternion2euler(goal_places[i].orientation)
                self.base.set_joint_value_target("odom_r", yaw)
            
                stats = False
                while not stats:
                    stats = self.base.go()

        else:
            while not rospy.is_shutdown():
                goal_place = self.lib["tf"].get_pose_with_offset(
                    "odom",
                    frame,
                    "goal_place_" + str(self.tf_index),
                    Pose(position=Point(pose.x, pose.y, 0.0),
                         orientation=self.lib["tf"].euler2quaternion((0.0, 0.0, pose.theta))))
                self.tf_index += 1

                self.base.set_joint_value_target("odom_x", goal_place.position.x)
                self.base.set_joint_value_target("odom_y", goal_place.position.y)
                _, _, yaw = self.lib["tf"].quaternion2euler(goal_place.orientation)
                self.base.set_joint_value_target("odom_r", yaw)

                exec_motion_synthesis(pose, motion_synthesis_config)

                # FIXME: Temporary handling of stuck bug.
                cnt = 0
                stats = False   
                while not stats:
                    if cnt == 10:
                        self.set_base_acc(0.5)
                        rospy.sleep(1.0)
                    elif cnt > 30:
                        rospy.sleep(3.0)
                        print("Error")
                        stats = False
                        break
                    cnt += 1

                    stats = self.base.go()
                if stats:
                    break

        self.ac_motion_synthesis.wait_for_result()
        rospy.loginfo("[" + rospy.get_name() + "]: NavState.SUCCESS")
        self.set_base_vel()
        self.set_base_acc()

        return 

    def arm_neutral(self):
        """Transition to neutral posture.

        Returns:
            bool: Execution result.
        """
        self.arm.set_named_target("neutral")
        return self.arm.go()


    def arm_go(self):
        """Transition to go posture.

        Returns:
            bool: Execution result.
        """
        self.arm.set_named_target("go")
        return self.arm.go()

    def gripper_command(self, val, is_sync=True, time=1.0):
        """Change the gripper opening width to a specified value.

        Args:
            val (float): Opening width. (Effective range: -0.785[rad] to 1.047[rad])
            is_sync (bool, optional): Synchronous/asynchronous settings. Defaults to True.
            time (float, optional): Wait time, valid only when wait=True. Defaults to 1.0.

        Returns:
            bool: Execution result.
        """
        self.gripper.set_joint_value_target("hand_motor_joint", val)
        if is_sync:
            return self.gripper.go(wait=True)
        else:
            self.gripper.go(wait=False)
            rospy.sleep(time)
            self.gripper.stop()
            return True

    def head_tilt(self, val):
        """Change the head tilt joint.

        Args:
            val (float): Angle of head tilt. (Effective range: ??[rad] to ??[rad])

        Returns:
            bool: Execution result.
        """
        self.head.set_joint_value_target("head_tilt_joint", val)
        return self.head.go()