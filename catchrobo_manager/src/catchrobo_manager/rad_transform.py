#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy


class RadTransform:
    def __init__(self):
        ### y軸は二倍動く
        pitch = rospy.get_param("pulley/pitch")
        teeth = rospy.get_param("pulley/teeth")
        radius = pitch * teeth / (2 * math.pi)
        self._pulley_radius = [
            radius,
            radius,
            radius,
            1,
        ]

    def rad2robot_m(self, id, position):
        ret = position * self._pulley_radius[id]
        if id == 1:
            ret *= 2
        return ret

    #### robot座標系での[m] -> motor回転角度[rad]に変換. gripperは入力をそのまま返す
    def robot_m2rad(self, motor_id, position):
        ret = position / self._pulley_radius[motor_id]
        if motor_id == 1:
            ### y軸は機構的に二倍動く
            ret *= 0.5
        return ret

    #### motor回転角度[rad] -> rviz表示用の値に変換　gripperは入力をそのまま返す
    def rad2rviz_joint_state(self, motor_id, rad):
        return rad * self._pulley_radius[motor_id]

    def get_pulley_radius(self, id):
        return self._pulley_radius[id]


### test 用
if __name__ == "__main__":
    transform = RadTransform()
    ret = transform.robot_m2rad(2, 0.063 - 0.00445)
    print(ret)
