#!/usr/bin/env python
# -*- coding: utf-8 -*-




#==================================================

## @file libsm.py
## @author Yutaro ISHIDA
## @brief ステートマシンライブラリクラス

#==================================================




#==================================================

# import

#==================================================
import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir("hma_ii_pkg") + "/script/import")
from common_import import *
from ros_import import *

import inspect

import tty
import termios




#==================================================

# グローバル

#==================================================




#==================================================

## @class LibSM
## @brief ステートマシンライブラリクラス

#==================================================
class LibSM:
    #==================================================
    
    ## @fn __init__
    ## @brief コンストラクタ
    ## @param
    ## @return

    #==================================================
    def __init__(
        self
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


        #==================================================

        # ROSインタフェース

        #==================================================


        #==================================================

        # イニシャライズ

        #==================================================
        self.add()


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
    
    ## @fn add
    ## @brief 追加関数
    ## @param
    ## @return 0 成功
    ## @return 1 失敗

    #==================================================
    def add(
        self
    ):
        return 0




    #==================================================

    ## @fn getKeyUnbuffer
    ## @brief バッファ無しで標準入力する関数
    ## @param
    ## @return key 標準入力

    #==================================================
    def getKeyUnbuffer(
        self
    ):
        buf_fd = sys.stdin.fileno()
        buf_setting = termios.tcgetattr(buf_fd)

        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(
                buf_fd,
                termios.TCSADRAIN,
                buf_setting
            )

        return key




    #==================================================

    ## @fn getClassName
    ## @brief オブジェクトのクラス名を取得する関数
    ## @param obj オブジェクト
    ## @return class_name クラス名

    #==================================================
    def getClassName(
        self,
        obj
    ):
        class_name = map(
            lambda x:x[0],
            inspect.getmembers(
                obj,
                inspect.isclass
            )
        )

        return class_name




    #==================================================

    ## @fn sortClassName
    ## @brief クラス名を記述順にソートする関数
    ## @param obj オブジェクト
    ## @param class_name クラス名
    ## @return sorted_class_name ソートされたクラス名

    #==================================================
    def sortClassName(
        self,
        obj,
        class_name
    ):
        lineno_and_class_name = []
        lineno_and_class_name.append([0, "Default"])
        for i in xrange(len(class_name)):
            attr = getattr(obj, class_name[i])
            if attr.__module__ == obj.__name__:
                class_instance = attr()
                lineno_and_class_name.append([class_instance._lineno, class_name[i]])
                del class_instance

        lineno_and_class_name = sorted(lineno_and_class_name, key = lambda x:x[0])
        sorted_class_name = []
        for i in xrange(len(lineno_and_class_name)):
            sorted_class_name.append(lineno_and_class_name[i][1])

        return sorted_class_name




    #==================================================

    ## @fn selectStartState
    ## @brief スタートステートを選択する関数
    ## @param sorted_class_name ソートされたクラス名
    ## @return start_state スタートステート

    #==================================================
    def selectStartState(
        self,
        sorted_class_name
    ):
        start_state = sorted_class_name[0]
        while not rospy.is_shutdown():
            for i in xrange(len(sorted_class_name)):
                sys.stdout.write("\r{0:^20}".format(sorted_class_name[i]))
                sys.stdout.flush()

                while not rospy.is_shutdown():
                    key = self.getKeyUnbuffer()
                    if key.encode("hex") in [b"0d", b"20", b"64"]: #Enter/Space/d
                        break

                if key.encode("hex") in [b"0d"]: #Enter
                    start_state = sorted_class_name[i]
                    break

                if key.encode("hex") in [b"64"]: #d
                    break
            else:
                continue

            print "\n"
            
            return start_state




    #==================================================

    ## @fn getStartState
    ## @brief スタートステートを取得する関数
    ## @param obj オブジェクト
    ## @return start_state スタートステート

    #==================================================
    def getStartState(
        self,
        obj
    ):
        class_name = self.getClassName(obj)
        sorted_class_name = self.sortClassName(obj, class_name)
        start_state = self.selectStartState(sorted_class_name)

        return start_state
