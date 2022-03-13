#!/usr/bin/env python3
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

import os
import sys
import numpy as np
import threading
import copy
import roslib
import rospy
import actionlib
import message_filters

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo

from hma_yolact_msgs.msg import YolactWithoutPose, YolactWithPose
from hma_yolact_action.msg import YolactAction, YolactResult

sys.path.remove("/opt/ros/melodic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge
import image_geometry

import hma_yolact_edge.hma_yolact as yolact
from hma_yolact_edge.data.hma_config import set_hma_cfg


# Global
GP_LOOP_RATE = 30.0
G_MM2M = 0.001 # meters to millimeters


class YolactServer:
    """YOLACT ROS Action server."""
    def __init__(self):
        self.lock = threading.Lock()

        self.init_ros_time = rospy.Time.now()
        self.update_ros_time = {}
        self.prev_ros_time = self.init_ros_time

        self.is_init = False

        self.smi_rgb = Image()
        self.smi_d = Image()
        self.cb = CvBridge()
        self.camera_model = None

        # ROS I/F
        self.p_action_name = rospy.get_param(rospy.get_name() + "/action_name", "/yolact")
        self.p_use_gpu = rospy.get_param(rospy.get_name() + "/use_gpu", True)
        self.p_cfg = rospy.get_param(rospy.get_name() + "/cfg", "yolact_edge_pytorch_config")
        self.p_weight = rospy.get_param(rospy.get_name() + "/weight", 
            roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/io/weights/yolact_edge_54_800000.pth")
        self.p_th = rospy.get_param(rospy.get_name() + "/th", 0.1)
        self.p_top_k = rospy.get_param(rospy.get_name() + "/top_k", 30)
        self.p_nms_th = rospy.get_param(rospy.get_name() + "/nms_th", 0.5)
        self.p_area_size_range = [rospy.get_param(rospy.get_name() + "/area_size_range/min", -1),
                                  rospy.get_param(rospy.get_name() + "/area_size_range/max", np.inf)]
        self.p_use_d = rospy.get_param(rospy.get_name() + "/use_d", True)
        self.p_rgb = rospy.get_param(rospy.get_name() + "/rgb", "/camera/rgb/image_raw")
        self.p_d = rospy.get_param(rospy.get_name() + "/d", "/camera/depth_registered/image_raw")
        self.p_camera_info = rospy.get_param(rospy.get_name() + "/camera_info", "/camera/depth_registered/camera_info")
        self.p_get_pose = rospy.get_param(rospy.get_name() + "/get_pose", False)
        self.p_frame = rospy.get_param(rospy.get_name() + "/frame", "/camera_frame")
        self.p_max_distance = rospy.get_param(rospy.get_name() + "/max_distance", -1.0)
        self.p_specific_id = rospy.get_param(rospy.get_name() + "/specific_id", "")
        self.p_ignore_id = rospy.get_param(rospy.get_name() + "/ignore_id", "")
        self.p_show_name = rospy.get_param(rospy.get_name() + "/show_name", False)

        self.pub_smi_dbg_rgb = rospy.Publisher(
            rospy.get_name() + "/dbg/rgb",
            Image,
            queue_size = 1)
        self.pub_smi_dbg_d = rospy.Publisher(
            rospy.get_name() + "/dbg/d",
            Image,
            queue_size = 1)
        self.pub_smi_dbg_mask = rospy.Publisher(
            rospy.get_name() + "/dbg/mask",
            Image,
            queue_size = 1)
        self.pub_yolact_without_pose = rospy.Publisher(
            rospy.get_name() + "/yolact_without_pose",
            YolactWithoutPose,
            queue_size = 1)

        self.sub_smi_rgb = message_filters.Subscriber(self.p_rgb, Image)
        self.sub_smi_d = message_filters.Subscriber(self.p_d, Image)
        interface = [self.sub_smi_rgb]
        if self.p_use_d:
            interface.append(self.sub_smi_d)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            interface, 10, 0.1, allow_headerless=True)
        if self.p_use_d:
            self.sync.registerCallback(self.subf_rgbd)
        else:
            self.sync.registerCallback(self.subf_rgb)

        if self.p_use_d and self.p_get_pose:
            self.pub_yolact_with_pose = rospy.Publisher(
                rospy.get_name() + "/yolact_with_pose",
                YolactWithPose,
                queue_size = 1)
            camera_info = rospy.wait_for_message(self.p_camera_info, CameraInfo)
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(camera_info)
            
        # Initialize
        self.yolact = yolact.YolactDetector(
            self.p_use_gpu, self.p_cfg, self.p_weight, self.p_th, self.p_top_k)
        self.yolact.setup()

        self.as_yolact = actionlib.SimpleActionServer(
            self.p_action_name,
            YolactAction,
            self.main
        )
        self.as_yolact.start()

        self.cfg = set_hma_cfg(self.p_cfg)

        self.is_init = True
        return

    def delete(self):
        return

    def subf_rgbd(self, smi_rgb, smi_d):
        """Callback function for RGB and Depth images.

        Args:
            smi_rgb (sensor_msg/Image): RGB image.
            smi_d (sensor_msg/Image): Depth image.
        """
        self.lock.acquire()
        self.smi_rgb = smi_rgb
        self.smi_d = smi_d
        self.lock.release()

        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return

    def subf_rgb(self, smi_rgb):
        """Callback function for RGB image.

        Args:
            smi_rgb (sensor_msg/Image): RGB image.
        """
        self.lock.acquire()
        self.smi_rgb = smi_rgb
        self.lock.release()

        fn = sys._getframe().f_code.co_name
        self.update_ros_time[fn] = rospy.Time.now()
        return

    def smi2cv(self, smi, encode, is_depth=False):
        """Convert from sensor_msgs/Image to OpenCV image.

        Args:
            smi (sensor_msgs/Image): Image before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.
            is_depth (bool, optional): Whether to convert depth image. Defaults to False.

        Returns:
            ndarray: Converted OpenCV image.
            If False is returned, the conversion has failed.
        """
        try:
            if is_depth:
                cv_tmp = self.cb.imgmsg_to_cv2(smi, encode)
                cv = np.array(cv_tmp, dtype=np.uint16)
            else:
                cv = self.cb.imgmsg_to_cv2(smi, encode)
            return cv
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def cv2smi(self, cv, encode):
        """Convert from OpenCV image to sensor_msgs/Image.

        Args:
            cv (ndarray): OpenCV image before conversion.
            encode (str): Conversion encode.
                mono8: grayscale image.
                mono16 16-bit grayscale image.
                bgr8: color image with blue-green-red color order.
                rgb8: color image with red-green-blue color order.
                bgra8: BGR color image with an alpha channel.
                rgba8: RGB color image with an alpha channel.

        Returns:
            sensor_msgs/Image: converted Image.
            If False is returned, the conversion has failed.
        """
        try:
            return self.cb.cv2_to_imgmsg(cv, encode)
        except:
            rospy.logwarn("[" + rospy.get_name() + "]: CV bridge FAILURE")
            return False

    def get_new_image(self, time, is_d):
        """Get a new images.

        Args:
            time (ROS time): Get images acquired at a time newer than "time."
            is_d (bool): Whether or not to get the depth image.
        
        Returns:
            sensor_msgs/Image: RGB and Depth image.
        """
        while not rospy.is_shutdown():
            fn = "subf_rgbd" if is_d else "subf_rgb"
            if fn not in self.update_ros_time.keys():
                continue
            if self.update_ros_time[fn] > time:
                self.lock.acquire()
                break

        return self.smi_rgb, self.smi_d
    
    def return_failure(self, message):
        """Return a failure.

        Args:
            message (str): Warning messages to be output.
        """
        rospy.logwarn("[" + rospy.get_name() + "]: " + message)

        result = YolactResult()
        result.result = False
        self.as_yolact.set_succeeded(result)
        
        self.lock.release()
        return

    def delete_unnecessary_result(self, cv_rgb, cv_d, classes, scores, boxes, masks):
        """Delete unnecessary recognition results.

        Using max distance (if depth image is available), ignore id, specific id, NMS.

        Args:
            cv_rgb (ndarray): RGB image.
            cv_d (ndarray): Depth image.
            classes (list[str]): Classes.
            scores (list[float]): Socres.
            boxes (list[list[int]]): Boxes.
            masks (list[list[float]]): Masks.

        Returns:
            list[str], list[float], list[list[int]], list[list[float]]: New lists with unnecessary results removed.
        """
        new_classes, new_scores, new_boxes, new_masks = [], [], [], []
        cv_pre = np.zeros(cv_rgb.shape[:2], np.uint8)
        cv_post = np.zeros(cv_rgb.shape[:2], np.uint8)
        for i in range(len(classes)):
            if self.cfg.dataset.class_names[classes[i]] in self.p_ignore_id.split(", "):
                continue

            if self.p_use_d and self.p_max_distance > 0:
                x1, y1, x2, y2 = boxes[i]
                mask = np.zeros((cv_d.shape[:2]), np.uint8)
                mask[:, :] = np.where(cv_d * G_MM2M > self.p_max_distance, 0, 255)
                cx, cy = int(x1 + (x2 - x1) / 2.0), int(y1 + (y2 - y1) / 2.0)
                if mask[cy, cx] == 0:
                    continue

            if (np.count_nonzero(masks[i]) > self.p_area_size_range[0] and \
                np.count_nonzero(masks[i]) < self.p_area_size_range[1]):
                if self.p_specific_id == "":
                    mask = masks[i].astype(np.uint8) * 255
                    cv_post = cv2.bitwise_or(cv_pre, mask)
                    cv_overlap = cv2.bitwise_and(cv_pre, cv_post)
                    cv_overlap_obj = cv2.bitwise_and(cv_overlap, mask)
                    if cv2.countNonZero(cv_overlap_obj) < cv2.countNonZero(mask) * self.p_nms_th:
                        cv_pre = copy.deepcopy(cv_post)
                        new_classes.append(classes[i])
                        new_scores.append(scores[i])
                        new_boxes.append(boxes[i])
                        new_masks.append(masks[i])
                elif self.cfg.dataset.class_names[classes[i]] in self.p_specific_id.split(", "):
                        mask = masks[i].astype(np.uint8) * 255
                        cv_post = cv2.bitwise_or(cv_pre, mask)
                        cv_overlap = cv2.bitwise_and(cv_pre, cv_post)
                        cv_overlap_obj = cv2.bitwise_and(cv_overlap, mask)
                        if cv2.countNonZero(cv_overlap_obj) < cv2.countNonZero(mask) * self.p_nms_th:
                            cv_pre = copy.deepcopy(cv_post)
                            new_classes.append(classes[i])
                            new_scores.append(scores[i])
                            new_boxes.append(boxes[i])
                            new_masks.append(masks[i])

        return new_classes, new_scores, new_boxes, new_masks

    def draw_mask(self, cv_src, mask, color, alpha=0.5, only_contour=True):
        """Drawing mask information on the input image.

        Args:
            cv_src (ndarray): Input image.
            mask (_type_): Mask information.
            color (_type_): Color of mask.
            alpha (float, optional): Transparency of mask. Defaults to 0.5.
            only_contour (bool, optional): Drawing only contours.
                The mask drawing process is too slow, so True is RECOMMENDED.
                Defaults to True.

        Returns:
            ndarray: Output image after drawing.
        """
        cv_dst = cv_src.copy()
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cv_dst, contours, -1, color=color, thickness=2)

        if not only_contour:
            for c in range(cv_dst.shape[2]):
                cv_dst[:, :, c] = np.where(mask > 0,
                                           cv_dst[:, :, c] * (1 - alpha) + alpha * color[c],
                                           cv_dst[:, :, c])

        return cv_dst

    def draw_result(self, cv_rgb, cv_d, classes, boxes, masks):
        """"Drawing result on the input image.

        Args:
            cv_src (ndarray): Input image.
            classes (list[str]): Classes.
            boxes (list[list[int]]): Boxes.
            masks (list[list[float]]): Masks.

        Returns:
            ndarray: Output image after drawing.
        """
        cv_dst = cv_rgb.copy()
        if self.p_use_d and self.p_max_distance > 0:
            mask = np.zeros((cv_d.shape[:2]), np.uint8)
            mask[:, :] = np.where(cv_d * G_MM2M > self.p_max_distance, 0, 255)
            cv_d_mask = cv2.merge([mask, mask, mask])
            cv_dst = cv2.bitwise_and(cv_dst, cv_d_mask)

        np.random.seed(42)
        color_masks = [np.random.randint(0, 256, (1, 3), dtype=np.uint8) for _ in range(max(classes) + 1)]
        for i in range(len(classes)):
            x1, y1, x2, y2 = boxes[i]

            cv_dst = self.draw_mask(cv_dst, masks[i], color_masks[i][0].tolist())
            cv2.rectangle(cv_dst, (x1, y1), (x2, y2), color_masks[i][0].tolist(), 1)

            if self.p_show_name:
                _class = self.cfg.dataset.show_names[classes[i]]
                font_scale = 0.4
            else:
                _class = self.cfg.dataset.class_names[classes[i]]
                font_scale = 0.6
            text_str = "%s" % (_class)

            font_face = cv2.FONT_HERSHEY_DUPLEX
            font_thickness = 1
            text_w, text_h = cv2.getTextSize(text_str, font_face, font_scale, font_thickness)[0]
            text_pt = (x1, y1 - 3)
            text_color = [255, 255, 255]

            cv2.rectangle(cv_dst, (x1, y1), (x1 + text_w, y1 - text_h - 4), color_masks[i][0].tolist(), -1)
            cv2.putText(cv_dst, text_str, text_pt, font_face, font_scale, text_color, font_thickness, cv2.LINE_AA)

        return cv_dst

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_yolact_action/YolactAction): Input informations.
        """
        # Check if initialization is finished
        if self.is_init == False:
            result = YolactResult()
            result.result = False
            self.as_yolact.set_succeeded(result)
            return

        # Set maximum recognition distance
        self.p_max_distance = rospy.get_param(rospy.get_name() + "/max_distance", -1)
        if goal.max_distance != -1:
            self.p_max_distance = goal.max_distance

        # Get image
        current_ros_time = rospy.Time.now()
        smi_rgb, smi_d = goal.rgb, goal.d
        if smi_rgb == Image():
            self.p_use_d = rospy.get_param(rospy.get_name() + "/use_d", True)
            self.p_get_pose = rospy.get_param(rospy.get_name() + "/get_pose", False)
            smi_rgb, smi_d = self.get_new_image(current_ros_time, self.p_use_d)
        else:
            self.lock.acquire()
            if smi_d == Image():
                self.p_use_d = False
                self.p_get_pose = False
            else:
                self.p_use_d = True

        # Convert from smi to cv
        try:
            cv_rgb = self.smi2cv(smi_rgb, "bgr8")
        except:
            self.return_failure("CV bridge FAILURE")
            return
            
        cv_d = Image()
        if self.p_use_d:
            try:
                cv_d = self.smi2cv(self.smi_d, "passthrough", True)
            except:
                self.return_failure("CV bridge FAILURE")
                return

        # Inference
        cv_result, classes, scores, boxes, masks = self.yolact.detect(cv_rgb)
        if len(classes) < 1 or len(scores) < 1 or len(boxes) < 1 or len(masks) < 1:
            self.pub_smi_dbg_rgb.publish(smi_rgb)
            self.return_failure("Object not detected.")
            return

        # Delete unnecessary results
        new_classes, new_scores, new_boxes, new_masks = self.delete_unnecessary_result(cv_rgb, cv_d, classes, scores, boxes, masks)
        new_classes = np.array(new_classes)
        new_scores = np.array(new_scores)

        if len(new_classes) < 1 or len(new_scores) < 1 or len(new_boxes) < 1 or len(new_masks) < 1:
            self.pub_smi_dbg_rgb.publish(self.smi_rgb)
            self.return_failure("Object not detected (due to delete unnecessary result).")
            return
            
        # Drawing results
        cv_new_result = self.draw_result(cv_result, cv_d, new_classes, new_boxes, new_masks, )

        # Output without pose
        yolact_without_pose = YolactWithoutPose()
        yolact_without_pose.rgb = smi_rgb
        yolact_without_pose.use_d = self.p_use_d
        if self.p_use_d:
            yolact_without_pose.d = smi_d
        yolact_without_pose.amount = len(new_classes)
        yolact_without_pose.id = [str(self.cfg.dataset.class_names[x]) for x in new_classes]
        yolact_without_pose.score = new_scores.tolist()
        yolact_without_pose.x = [box[0] for box in (new_boxes)]
        yolact_without_pose.y = [box[1] for box in (new_boxes)]
        yolact_without_pose.w = [abs(box[2] - box[0]) for box in (new_boxes)]
        yolact_without_pose.h = [abs(box[3] - box[1]) for box in (new_boxes)]

        mask = new_masks[0]
        if len(new_classes) > 1:
            for i in range(len(new_classes) - 1):
                mask = cv2.bitwise_or(mask, new_masks[i + 1] * (i + 2))
        try:
            yolact_without_pose.mask = self.cv2smi(mask.astype(np.uint8), "mono8")
            smi_dbg_mask = self.cv2smi(mask.astype(np.uint8) * 255, "mono8")
        except:
            self.return_failure("CV bridge FAILURE")
            return

        # Output with pose
        if self.p_use_d and self.p_get_pose:
            yolact_with_pose = YolactWithPose()
            for i in range(len(new_classes)):
                # Calculate the centroid
                try:
                    mu = cv2.moments(new_masks[i]*255, False)
                    centroid = (int(mu["m10"] / mu["m00"]), int(mu["m01"] / mu["m00"]))
                    cv2.circle(cv_new_result, centroid, 2, (0, 0, 255), -1)
                except ZeroDivisionError:
                    yolact_with_pose.pose.append(Pose())
                    yolact_with_pose.is_valid_pose.append(False)
                    continue

                # Get depth around the centroid
                kernel = [-1, 0, 1] 
                depth_list = []
                for y in kernel:
                    for x in kernel:
                        depth = cv_d[centroid[1] + y, centroid[0] + x] * G_MM2M
                        if depth > 0:
                            depth_list.append(depth)
                            
                # Calculate 3D coordinates
                if len(depth_list) != 0:
                    uv = list(self.camera_model.projectPixelTo3dRay((centroid[0], centroid[1])))
                    uv[:] = [x / uv[2] for x in uv]
                    uv[:] = [x * np.mean(depth_list) for x in uv]
                else:
                    yolact_with_pose.pose.append(Pose())
                    yolact_with_pose.is_valid_pose.append(False)
                    continue

                yolact_with_pose.pose.append(Pose(Point(uv[0], uv[1], uv[2]), Quaternion(0, 0, 0, 1)))
                yolact_with_pose.is_valid_pose.append(True)

            yolact_with_pose.rgb = yolact_without_pose.rgb
            yolact_with_pose.use_d = yolact_without_pose.use_d
            yolact_with_pose.d = yolact_without_pose.d
            yolact_with_pose.amount = yolact_without_pose.amount
            yolact_with_pose.id = yolact_without_pose.id
            yolact_with_pose.score = yolact_without_pose.score
            yolact_with_pose.x = yolact_without_pose.x
            yolact_with_pose.y = yolact_without_pose.y
            yolact_with_pose.w = yolact_without_pose.w
            yolact_with_pose.h = yolact_without_pose.h
            yolact_with_pose.mask = yolact_without_pose.mask
            yolact_with_pose.frame = str(self.p_frame)

        # Publish
        try:
            smi_dbg_rgb = self.cv2smi(cv_new_result, "bgr8")
        except:
            self.return_failure("CV bridge FAILURE")
            return

        self.pub_smi_dbg_rgb.publish(smi_dbg_rgb)
        self.pub_smi_dbg_d.publish(smi_d)
        self.pub_smi_dbg_mask.publish(smi_dbg_mask)

        result = YolactResult()
        result.result = True
        result.without_pose = yolact_without_pose

        if self.p_use_d and self.p_get_pose:
            result.with_pose = yolact_with_pose

        self.as_yolact.set_succeeded(result)

        self.lock.release()
        return


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = YolactServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()