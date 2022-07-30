#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.gui_menu_enum import CalibrationMenu

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from std_msgs.msg import Int8


import math


class Calibration:
    def __init__(self):
        self.FIELD = rospy.get_param("field")
        name_space = "calibration/"
        self.BASE_FRAME = rospy.get_param(name_space + "base_frame")
        self.SHOOTING_BOX_REAL_FRAME = rospy.get_param(
            name_space + "shooting_box_real_frame")
        self.SHOOTING_BOX_IDEAL_FRAME = rospy.get_param(
            name_space + "shooting_box_ideal_frame")
    
        shooting_box_center_red = rospy.get_param(
            name_space + "shooting_box_center_red")
        self._br = tf2_ros.StaticTransformBroadcaster()
        self.update_tf("shooting_box_ideal_frame",
                       shooting_box_center_red[0], shooting_box_center_red[1], 0)
        rospy.sleep(0.5)
        self.update_tf(self.SHOOTING_BOX_REAL_FRAME,
                       shooting_box_center_red[0] + 0.1, shooting_box_center_red[1] + 0.3, 0)
        self._gui_msg = CalibrationMenu.NONE

        self._edges = [None] * 4

        rospy.Subscriber("calibration_menu", Int8, self.gui_callback, queue_size=3)

    def gui_callback(self, msg):
        self._gui_msg = msg

        ### [TODO] 4辺の情報が集まっていたらtfを更新
        for val in self._edges:
            ### 4辺の情報がどれかまだ取れていなければreturn
            if val is None:
                return

        # if self._gui_msg ==
        self.calc_rectangle()
        self.update_tf()

    def robot_position_callback(self):
        ### [TODO] 測定中なら、現在値を対象の辺に追加

        pass

    def calc_rectangle(self):
        ### [TODO] 取得した点から画像を作成

        ### [TODO] 四角形(矩形)抽出. 中心位置、回転を取得
        pass

    def update_tf(self, frame_id, x_m, y_m, theta):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.BASE_FRAME
        t.child_frame_id = frame_id

        t.transform.translation.x = x_m
        t.transform.translation.y = y_m
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("calibration")
    calibration = Calibration()
    rospy.spin()
