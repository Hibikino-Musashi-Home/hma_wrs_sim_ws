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
import collections

sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

sys.path.append(roslib.packages.get_pkg_dir("hma_lib_pkg") + "/script/src/lib")
from libutil import *
from libtf import *

sys.path.append(roslib.packages.get_pkg_dir("hma_yolact_pkg") + "/script/src/lib")
from libyolact import *

from sklearn.cluster import DBSCAN


# Global
GP_LOOP_RATE = 30.0


class MappingObjectServer:
    """Object mapping ROS Action server."""
    def __init__(self):
        self.update_ros_time = {}

        self.libutil = LibUtil()
        self.libtf = LibTF()
        self.libyolact = LibYolact()

        self.lib = {"util": self.libutil, "tf": self.libtf, "yolact": self.libyolact}
        self.robot_descriptor = self.lib["util"].get_robot_descriptor()

        self.index = 0
        self.objects = np.empty((0, 5), float) # id, pose(x, y, z), score
        self.object_names = {-1: "name"}
        self.mapping_data = {"id":[], "place":[], "pose":[], "score":[]}

        # ROS I/F
        self.p_area_data = rospy.get_param(
            rospy.get_name() + "/area_data",
            roslib.packages.get_pkg_dir("hma_common_pkg") + "/io/mapping_area/area.json")
        self.p_select_mode = rospy.get_param(rospy.get_name() + "/select_mode", 1)
        self.p_frame = rospy.get_param(rospy.get_name() + "/frame", "map")

        self.pub_area = rospy.Publisher(rospy.get_name() + "/area", Marker, queue_size = 1)
        self.pub_points = rospy.Publisher(rospy.get_name() + "/points", Marker, queue_size = 1)
        self.pub_text = rospy.Publisher(rospy.get_name() + "/text", Marker, queue_size = 10)

        self.as_mapping_object = actionlib.SimpleActionServer("/mapping_object",
                                                              MappingObjectAction,
                                                              self.main)
        self.as_mapping_object.start()

        # Set area markers
        rospy.sleep(1.0) # Do not delete this because the marker will not appear
        json_open = open(self.p_area_data, "r")
        self.area_data = json.load(json_open)
        self.set_mapping_area_marker(self.area_data)

        return

    def delete(self):
        for key in self.lib.keys():
            self.lib[key].delete()

        # Delete markers
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.pub_area.publish(marker)
        self.pub_points.publish(marker)
        self.pub_text.publish(marker)

        return

    def set_mapping_area_marker(self, area_data):
        """Set markers for mapping area.

        Args:
            area_data (dict[str, str, str, float]): Data of the mapping area.
                e.g. {"place": "min": "x": (value)}
        """
        for i, key in enumerate(area_data.keys()):
            marker = Marker()
            marker.header.frame_id = self.p_frame
            marker.header.stamp = rospy.Time.now()
            marker.lifetime = rospy.Duration()
            marker.ns = key
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.scale.x = 0.01
            marker.pose.orientation.w = 1.0
            
            # points
            x_list = [area_data[key]["min"]["x"], area_data[key]["max"]["x"]]
            y_list = [area_data[key]["min"]["y"], area_data[key]["max"]["y"]]
            z_list = [area_data[key]["min"]["z"], area_data[key]["max"]["z"]]
            for x in x_list:
                for y in y_list:
                    for z in z_list:
                        marker.points.append(Point(x, y, z))
            for z in z_list:
                for x in x_list:
                    for y in y_list:
                        marker.points.append(Point(x, y, z))
            for y in y_list:
                for z in z_list:
                    for x in x_list:
                        marker.points.append(Point(x, y, z))

            self.pub_area.publish(marker)
            rospy.sleep(0.1)

        return

    def set_points_and_text_marker_from_mapping_data(self, data):
        """Set points and text markers.

        Args:
            data (dict): Mapping data.
        """
        # Delete markers
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.pub_points.publish(marker)
        self.pub_text.publish(marker)

        # Points
        marker_points = Marker()
        marker_points.header.frame_id = self.p_frame
        marker_points.header.stamp = rospy.Time.now()
        marker_points.lifetime = rospy.Duration()
        marker_points.ns = "points"
        marker_points.id = 0
        marker_points.type = Marker.SPHERE_LIST
        marker_points.action = Marker.ADD
        marker_points.color.r = 1.0
        marker_points.color.g = 0.2
        marker_points.color.b = 0.0
        marker_points.color.a = 1.0
        marker_points.scale.x = 0.05
        marker_points.scale.y = 0.05
        marker_points.scale.z = 0.05
        marker_points.pose.orientation.w = 1.0

        # Text
        marker_text = Marker()
        marker_text.header.frame_id = self.p_frame
        marker_text.header.stamp = rospy.Time.now()
        marker_text.lifetime = rospy.Duration()
        marker_text.ns = "text"
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.color.r = 1.0
        marker_text.color.g = 0.2
        marker_text.color.b = 0.0
        marker_text.color.a = 1.0
        marker_text.scale.z = 0.1
        marker_text.pose.orientation.w = 1.0

        # Publish
        for i in range(len(data["pose"])):
            marker_points.points.append(data["pose"][i].position)
            marker_text.id = i
            marker_text.pose.position = Point(data["pose"][i].position.x,
                                              data["pose"][i].position.y,
                                              data["pose"][i].position.z + 0.1)
            marker_text.text = self.object_names[int(data["id"][i])]
            self.pub_text.publish(marker_text)

        self.pub_points.publish(marker_points)
        return

    def add_object(self, area_data, ids, names, poses, scores, search_place):
        """Function to add objects for mapping.

        Args:
            area_data (dict[str, str, str, float]): Data of the mapping area.
                e.g. {"place": "min": "x": (value)}
            ids (list[str]): List of object id.
            names (list[str]): List of object name.
            poses (list[sensor_msgs/Pose]): List of object pose.
            scores (list[float]): List of object score.
            search_place (str): Limitations of search place.
        """
        for i in range(len(ids)):
            for key in area_data.keys():
                if search_place != "" and search_place not in key:
                    continue
                if poses[i].position.x < area_data[key]["min"]["x"] or poses[i].position.x > area_data[key]["max"]["x"]: 
                    continue
                if poses[i].position.y < area_data[key]["min"]["y"] or poses[i].position.y > area_data[key]["max"]["y"]: 
                    continue
                if poses[i].position.z < area_data[key]["min"]["z"] or poses[i].position.z > area_data[key]["max"]["z"]: 
                    continue

                self.objects = np.append(self.objects,
                                         np.array([[float(ids[i]),
                                                    poses[i].position.x,
                                                    poses[i].position.y,
                                                    poses[i].position.z,
                                                    scores[i]]]),
                                         axis = 0)
                self.object_names[int(ids[i])] = names[i]

        return

    def mapping(self, area_data, objects):
        """Function to perform mapping.

        Args:
            area_data (dict[str, str, str, float]): Data of the mapping area.
                e.g. {"place": "min": "x": (value)}
            objects (ndarray): ID, pose and score information of mapping objects.
                e.g. [[id, pose_x, pose_y, pose_z, score], [...], ...]

        Returns:
            bool: Whether it has been mapping or not.
        """
        if objects == []:
            return False

        # Clustering
        try:
            model = DBSCAN(eps=0.1, min_samples=3)
            model.fit_predict(objects[:, 1:4])
            pred = model.fit_predict(objects[:, 1:4])
        except:
            return False

        n_cluster = len(set(model.labels_))
        rospy.loginfo("[" + rospy.get_name() + "]: N = " + str(n_cluster))
        if n_cluster == 0:
            return False

        # Formatting
        ids = []
        points = []
        sum_points = []
        scores = []
        for i in range(n_cluster):
            if -1 in model.labels_:
                indices = [j for j, x in enumerate(model.labels_) if x+1 == i]
            else:
                indices = [j for j, x in enumerate(model.labels_) if x == i]

            if len(indices) != 0:
                if model.labels_[indices[0]] < 0:
                    continue

                ids.append([])
                sum_points.append(Point())
                scores.append({})
                for index in indices:
                    ids[-1].append(int(objects[:, 0][index]))
                    sum_points[-1].x += objects[:, 1][index]
                    sum_points[-1].y += objects[:, 2][index]
                    sum_points[-1].z += objects[:, 3][index]

                    if ids[-1][-1] not in scores[-1].keys():
                        scores[-1][ids[-1][-1]] = []
                    scores[-1][ids[-1][-1]].append(objects[:, 4][index])

                points.append(Point(sum_points[-1].x / float(len(indices)),
                                    sum_points[-1].y / float(len(indices)),
                                    sum_points[-1].z / float(len(indices))))

        for i, id_ in enumerate(ids):
            res_id = "-1"
            res_score = -1

            # Select a Mode ID
            if self.p_select_mode == 0:
                count = collections.Counter(id_)
                mc = count.most_common(1)
                res_id = mc[0][0]
                indices = [j for j, x in enumerate(id_) if x == res_id]
                if len(indices) != 0:
                    res_score = sum(scores[i][res_id]) / float(len(indices))

            # Select a highest score ID
            elif self.p_select_mode == 1:
                for key in scores[i].keys():
                    max_score = max(scores[i][key])
                    if max_score > res_score or int(res_id) == 999:
                        res_id = key
                        res_score = max_score

            # place
            for key in area_data.keys():
                if points[i].x < area_data[key]["min"]["x"] or points[i].x > area_data[key]["max"]["x"]: 
                    continue
                if points[i].y < area_data[key]["min"]["y"] or points[i].y > area_data[key]["max"]["y"]: 
                    continue
                if points[i].z < area_data[key]["min"]["z"] or points[i].z > area_data[key]["max"]["z"]: 
                    continue
                self.mapping_data["place"].append(key)
                break
            else:
                continue

            # id, pose, score
            self.mapping_data["id"].append(str(int(res_id)))
            self.mapping_data["pose"].append(Pose(points[i], Quaternion(0, 0, 0, 1)))
            self.mapping_data["score"].append(res_score)

            # marker
            self.set_points_and_text_marker_from_mapping_data(self.mapping_data)

        rospy.loginfo("[" + rospy.get_name() + "]: Clustring complete.")
        return True

    def get_min_distance_index(self, data, pose, search_place):
        min_d = np.inf
        index = -1
        for i, p in enumerate(data["pose"]):
            if search_place != "" and search_place not in data["place"][i]:
                continue
            d = math.sqrt(pow(p.position.x - pose.position.x, 2) + pow(p.position.y - pose.position.y, 2))
            if d > min_d:
                continue
            min_d = d
            index = i
        return min_d, index

    def main(self, goal):
        """Main function.

        Args:
            goal (hma_common_action/MappingObjectAction): Input informations.
        """
        result = MappingObjectResult()
        result.result = False

        # Add
        if goal.action == MappingAction.ADD:
            self.add_object(self.area_data, goal.id, goal.name, goal.pose, goal.score, goal.search_place)
            result.result = True

        # Mapping
        if goal.action == MappingAction.MAPPING:
            self.mapping_data = {"id":[], "place":[], "pose":[], "score":[]}
            result.result = self.mapping(self.area_data, self.objects)
            self.objects = np.empty((0, 5), float) # clear

        # Delete object
        elif goal.action == MappingAction.DELETE:
            d, index = self.get_min_distance_index(self.mapping_data, goal.pose[0], goal.search_place)
            try:
                txt = self.object_names[int(self.mapping_data["id"][index])]
                # print (index, txt, d)
            except:
                pass
            if index == -1: # TODO 調整
                result.result = False
            else:
                self.mapping_data["id"].pop(index)
                self.mapping_data["place"].pop(index)
                self.mapping_data["pose"].pop(index)
                self.mapping_data["score"].pop(index)
                self.set_points_and_text_marker_from_mapping_data(self.mapping_data)
                result.result = True

        # Delete all object
        elif goal.action == MappingAction.DELETEALL:
            self.mapping_data = {"id":[], "place":[], "pose":[], "score":[]}
            self.set_points_and_text_marker_from_mapping_data(self.mapping_data)
            result.result = True

        # Check
        elif goal.action == MappingAction.CHECK:
            for key in self.area_data.keys():
                if goal.pose[0].position.x < self.area_data[key]["min"]["x"] or goal.pose[0].position.x > self.area_data[key]["max"]["x"]: 
                    continue
                if goal.pose[0].position.y < self.area_data[key]["min"]["y"] or goal.pose[0].position.y > self.area_data[key]["max"]["y"]: 
                    continue
                if goal.pose[0].position.z < self.area_data[key]["min"]["z"] or goal.pose[0].position.z > self.area_data[key]["max"]["z"]: 
                    continue
                result.place = [key]

            d, index = self.get_min_distance_index(self.mapping_data, goal.pose[0], goal.search_place)
            if index == -1 or d > 0.05: # TODO 調整
                result.result = False
            else:
                result.amount = 1
                result.id = goal.id
                result.pose = goal.pose
                result.score = goal.score
                try:
                    if self.mapping_data["id"][index] != int(goal.id[0]):
                        if self.mapping_data["score"][index] < goal.score[0]:
                            result.id = [self.mapping_data["id"][index]]
                            result.pose = [self.mapping_data["pose"][index]]
                            result.place = [self.mapping_data["place"][index]]
                            result.score = [self.mapping_data["score"][index]]
                except:
                    pass
                result.result = True

        # Get object
        elif goal.action == MappingAction.GET: # TODO 
            result.result = True

        # Get all object
        elif goal.action == MappingAction.GETALL:
            result.amount = len(self.mapping_data["id"])
            result.id = self.mapping_data["id"]
            result.place = self.mapping_data["place"]
            result.pose = self.mapping_data["pose"]
            result.score = self.mapping_data["score"]
            result.result = True

        # Get nearest object
        elif goal.action == MappingAction.GETNEAREST:
            _, index = self.get_min_distance_index(self.mapping_data, goal.pose[0], goal.search_place)
            if index == -1:
                result.result = False
            else:
                result.amount = 1
                result.id = [self.mapping_data["id"][index]]
                try:
                    result.place = [self.mapping_data["place"][index]]
                except:
                    result.place = [""] # FIXME バグ
                result.pose = [self.mapping_data["pose"][index]]
                result.score = [self.mapping_data["score"][index]]
                result.result = True

        self.as_mapping_object.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", GP_LOOP_RATE)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = MappingObjectServer()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass 
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()
