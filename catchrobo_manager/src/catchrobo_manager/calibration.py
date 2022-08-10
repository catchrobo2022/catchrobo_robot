#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.jagarico.database import Database

import rospy
import tf
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int8


import numpy as np


class Calibration:
    def __init__(self) -> None:
        self.FIELD = rospy.get_param("field")
        name_space = "calibration/"
        self.BASE_FRAME = rospy.get_param(name_space + "base_frame")
        self.SHOOTING_BOX_REAL_FRAME = rospy.get_param(
            name_space + "shooting_box_real_frame"
        )
        self.TEMP_FILE = "temp/" + self.FIELD + "_shoot.csv"

        # self.SHOOTING_BOX_IDEAL_FRAME = rospy.get_param(
        #     name_space + "shooting_box_ideal_frame"
        # )

        # shooting_box_center_red = rospy.get_param(
        #     name_space + "shooting_box_center_red"
        # )
        database = Database()
        csv_name = self.FIELD + "_shoot.csv"
        database.readCsv(csv_name)
        self._database = database

        self.BOX_IDS = [0, 2, 15, 17]
        self._br = tf2_ros.StaticTransformBroadcaster()

        self._default_points, self._default_center = self.init_default_points()
        self._points = np.copy(self._default_points)

        position_topic = rospy.get_param("radius2meter/world_position_topic")
        self._pub_tf = rospy.Publisher("tf", TFMessage, queue_size=1)
        rospy.Subscriber(position_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber("menu", Int8, self.gui_callback, queue_size=1)

    #############################################################
    ### callback
    #############################################################
    def pose_callback(self, msg: PoseStamped) -> None:
        self._robot_pose = msg

    def gui_callback(self, msg) -> None:
        menu = msg.data
        # if GuiMenu.POINT1 <= menu < GuiMenu.INIT:
        ### [TODO] 名前変えてもらったほうが良さそう. END
        if GuiMenu.POINT1 <= menu <= GuiMenu.POINT4:
            ### point 指示なら、pointsに順に追加していく
            self.set_point(self._robot_pose)
        if menu == GuiMenu.POINT4:
            self.calibration()

    #############################################################
    ### function
    #############################################################

    def calibration(self) -> None:
        center, rotate = self.calc_tf()
        center[0] += 0.002
        # center[1] += 0.2
        # rotate = 0.01  # np.math.pi / 2
        self.pub_tf(
            self.SHOOTING_BOX_REAL_FRAME,
            center[0],
            center[1],
            rotate,
        )

        diff = center - self._default_center
        rospy.loginfo("diff {} rotate{}".format(diff, rotate))
        self.update_database(center, rotate)

    def update_database(self, center: np.ndarray, rotate: float) -> None:
        homogeneous_M = self.generate_homogeneous(center, rotate)
        for i in range(self._database.getIdNum()):
            default_point = self._database.getPosi(i)
            q_box = default_point - self._default_center
            # point = self._points[i]
            q = np.asarray([q_box[0], q_box[1], default_point[2], 1]).T
            transformed_q = homogeneous_M.dot(q)

            self._database.updateState(i, "x", transformed_q[0])
            self._database.updateState(i, "y", transformed_q[1])
            # self._database.updateState(i, "z", transformed_q[2])
        self._database.save_csv(self.TEMP_FILE)

    def init_default_points(self):
        ### default値の生成。
        shape = (len(self.BOX_IDS), 3)
        default_points = np.zeros(shape)
        for i, box_id in enumerate(self.BOX_IDS):
            position = self._database.getPosi(box_id)
            id = i
            default_points[id] = position

        center = self.get_center(default_points)

        ### 順番を入れ替える
        sorted_points = np.zeros(shape)
        for val in default_points:
            id = self.get_quadrant(val, center)
            sorted_points[id] = val
        return sorted_points, center

    def get_center(self, points: np.ndarray) -> np.ndarray:
        return np.average(points, axis=0)

    def get_quadrant(self, point: np.ndarray, center: np.ndarray) -> int:
        ### 象限で配列idを決める
        diff = point - center
        id = 0
        if diff[0] >= 0 and diff[1] >= 0:
            id = 0
        elif diff[0] < 0 and diff[1] >= 0:
            id = 1
        elif diff[0] < 0 and diff[1] < 0:
            id = 2
        else:
            id = 3
        return id

    def get_theta(self, point: np.ndarray, center: np.ndarray) -> np.ndarray:
        diff = point - center
        return np.arctan2(diff[1], diff[0])

    def set_point(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        point = [position.x, position.y, position.z]
        point = np.asarray(point)
        rospy.loginfo(point)
        id = self.get_quadrant(point, self._default_center)
        self._points[id] = point

    def calc_tf(self):
        points = self._points
        center = self.get_center(points)
        ### 角度の差分も平均で計算する
        diffs = np.zeros(len(self.BOX_IDS))

        for i in range(len(self.BOX_IDS)):
            # for i, point, default_point in enumerate(zip(points, self._default_points)):
            point = points[i]
            default_point = self._default_points[i]
            default_theta = self.get_theta(default_point, self._default_center)
            theta = self.get_theta(point, center)
            diffs[i] = theta - default_theta

        rotate = np.average(diffs)

        return center, rotate

    def pub_tf(self, frame_id, x_m, y_m, theta):
        t = TransformStamped()
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

        msg = TFMessage()
        msg.transforms = [t]
        self._pub_tf.publish(msg)

    def update_tf(self, frame_id, x_m, y_m, theta) -> None:
        ### 現在使っていない
        t = TransformStamped()
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

    def generate_homogeneous(self, center: np.ndarray, theta: float) -> np.ndarray:
        # Quat to euler sxyz とあるが， XYZWの順番で良い。ちょっとわかりにくくないか？
        tfeul = [0, 0, theta]
        # 並進量の記述
        tftrans = [center[0], center[1], 0]
        tfobjM = tf.transformations.compose_matrix(angles=tfeul, translate=tftrans)
        # return
        return tfobjM


if __name__ == "__main__":
    rospy.init_node("calibration")
    calibration = Calibration()
    rospy.spin()
