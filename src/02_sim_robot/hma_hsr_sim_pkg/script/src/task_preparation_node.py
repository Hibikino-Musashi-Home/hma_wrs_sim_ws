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




#==================================================

# import

#==================================================
import os
import sys
import roslib
import rospy
import threading
import random
import numpy
import copy
import tf

from subprocess import Popen, call

from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Pose
from gazebo_msgs.msg import ModelState, ModelStates

from gazebo_msgs.srv import SetModelState




#==================================================

# グローバル

#==================================================
GP_DYNAMIC_LOOP_RATE = False
GP_LOOP_RATE = 30.0




#==================================================

## @class TaskPreparationNode
## @brief タスク準備クラス

#==================================================
class TaskPreparationNode:
    #==================================================
    
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param run_enable 実行許可
    ## @return

    #==================================================
    def __init__(
        self,
        run_enable = True
    ):
        #==================================================

        # メンバ変数

        #==================================================
        self._lock = threading.Lock()

        self._init_ros_time = rospy.Time.now()
        self._update_ros_time = {}
        self._prev_ros_time = self._init_ros_time

        self._lib = {
        }

        self._run_enable = Bool()
        self._gazebo_model_states = ModelStates()
        self._gazebo_model_states_saved = None

        self._run_enable.data = run_enable


        #==================================================

        # ROSインタフェース

        #==================================================
        self._p_seed = rospy.get_param(
            rospy.get_name() + "/seed",
            ""
        )
        self._p_percategory = rospy.get_param(
            rospy.get_name() + "/percategory",
            ""
        )
        self._p_obstacles = rospy.get_param(
            rospy.get_name() + "/obstacles",
            ""
        )
        self._p_perrow = rospy.get_param(
            rospy.get_name() + "/perrow",
            ""
        )

        self._p_drawer = rospy.get_param(
            rospy.get_name() + "/drawer_open",
            True
        )

        self._p_task = rospy.get_param(
            rospy.get_name() + "/task",
            "task"
        )
        self._p_prep = rospy.get_param(
            rospy.get_name() + "/prep",
            ""
        )

        self._sub_gazebo_model_states = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.subfGazeboModelStates,
            queue_size = 1
        )


        #==================================================

        # イニシャライズ

        #==================================================


        return




    #==================================================
    
    ## @fn delete
    ## @brief デストラクタ
    ## @param
    ## @return

    #==================================================
    def delete(
        self
    ):
        #==================================================

        # ファイナライズ

        #==================================================
        for key in self._lib.keys():
            self._lib[key].delete()


        return




    #==================================================
    
    ## @fn subfRunEnable
    ## @brief 実行許可サブスクライブ関数
    ## @param run_enable 実行許可
    ## @return

    #==================================================
    def subfRunEnable(
        self,
        run_enable
    ):
        self._run_enable = run_enable
            

        return




    #==================================================
    
    ## @fn subfGazeboModelStates
    ## @brief Gazeboのモデルステータス取得関数
    ## @param gazebo_model_states モデルステータス
    ## @return

    #==================================================
    def subfGazeboModelStates(
        self,
        gazebo_model_states
    ):
        self._lock.acquire()
        self._gazebo_model_states = gazebo_model_states
        self._lock.release()
    
        fn = sys._getframe().f_code.co_name
        self._update_ros_time[fn] = rospy.Time.now()

        try:
            self.proc()
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")


        return



    #==================================================
    
    ## @fn deleteModels
    ## @brief DeleteModels関数
    ## @param 
    ## @return

    #==================================================
    def deleteModels(
        self
    ):
        for name in self._gazebo_model_states.name:
            if self._p_task in name:
                cmd = ['rosservice', 'call', 'gazebo/delete_model',
                    '{model_name: ' + str(name) + '}']
                call(cmd)

                rospy.loginfo("[" + rospy.get_name() + "]: Delete " + str(name))


        return




    #==================================================
    
    ## @fn closeDrawers
    ## @brief CloseDrawers関数
    ## @param 
    ## @return

    #==================================================
    def closeDrawers(
        self
    ):
        rospy.wait_for_service("/gazebo/set_model_state")
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.wait_for_service("/gazebo/set_model_state")

        point = [
            Point(-2.7, 0.67, 0.095),
            Point(-2.7, 1.0, 0.095),
            Point(-2.7, 1.0, 0.35)
        ]
        for i, name in enumerate(["trofast_1", "trofast_2", "trofast_3"]):
            model_state = ModelState()
            model_state.model_name = name
            model_state.pose = Pose(
                point[i],
                Quaternion(0, 0, 0, 1)
            )

            self._set_model_state(model_state)


        return




    #==================================================
    
    ## @fn openDrawers
    ## @brief OpenDrawers関数
    ## @param 
    ## @return

    #==================================================
    def openDrawers(
        self
    ):
        rospy.wait_for_service("/gazebo/set_model_state")
        self._set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.wait_for_service("/gazebo/set_model_state")

        point = [
            Point(-2.7 + 0.3, 0.67, 0.095),
            Point(-2.7 + 0.3, 1.0, 0.095),
            Point(-2.7 + 0.3, 1.0, 0.35)
        ]
        for i, name in enumerate(["trofast_1", "trofast_2", "trofast_3"]):
            model_state = ModelState()
            model_state.model_name = name
            model_state.pose = Pose(
                point[i],
                Quaternion(0, 0, 0, 1)
            )

            self._set_model_state(model_state)




    #==================================================
    
    ## @fn proc
    ## @berif 処理関数
    ## @param
    ## @return

    #==================================================
    def proc(
        self
    ):
        #==================================================

        # 実行許可の検証

        #==================================================
        if self._run_enable.data == False:
            return
        

        current_ros_time = rospy.Time.now()
        self._lock.acquire()


        #==================================================

        # タスクの準備

        #==================================================
        if self._p_prep:
            self.closeDrawers()
            self.deleteModels()

            if self._p_seed == "":
                cmd = "rosrun tmc_wrs_gazebo_worlds spawn_objects" \
                    + " --percategory " + str(self._p_percategory) \
                    + " --obstacles " + str(self._p_obstacles) \
                    + " --perrow " + str(self._p_perrow)
            else:
                cmd = "rosrun tmc_wrs_gazebo_worlds spawn_objects" \
                    + " --seed " + str(self._p_seed) \
                    + " --percategory "+ str(self._p_percategory) \
                    + " --obstacles " + str(self._p_obstacles) \
                    + " --perrow " + str(self._p_perrow)
            Popen(cmd.split(" "))

        rospy.sleep(30.0)
        if self._p_drawer:
            self.openDrawers()
        

        self._lock.release()
        self._prev_ros_time = current_ros_time
        
        self._run_enable.data = False
        

        return




#==================================================

# メイン

#==================================================
if __name__ == "__main__":
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(
        rospy.get_name() + "/loop_rate",
        GP_LOOP_RATE
    )
    loop_wait = rospy.Rate(p_loop_rate)

    cls = TaskPreparationNode()

    rospy.on_shutdown(cls.delete)

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            rospy.logerr("[" + rospy.get_name() + "]: FAILURE")
        loop_wait.sleep()