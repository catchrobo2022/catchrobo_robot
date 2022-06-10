#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

class RadTransform:
    def __init__(self):
        ### y軸は二倍動く
        self._pulley_radius = [0.002 * 54/(2*math.pi), 0.002 * 54/(2*math.pi),  0.002 * 54/(2*math.pi), 1]


    #### robot座標系での[m] -> motor回転角度[rad]に変換. gripperは入力をそのまま返す
    def robot_m2rad(self, motor_id, position):
        ret =  position / self._pulley_radius[motor_id] 
        if motor_id == 1:
            ### y軸は機構的に二倍動く
            ret *=0.5       
        return ret


    #### motor回転角度[rad] -> rviz表示用の値に変換　gripperは入力をそのまま返す
    def rad2rviz_joint_state(self,motor_id, rad):
        return rad * self._pulley_radius[motor_id]        

