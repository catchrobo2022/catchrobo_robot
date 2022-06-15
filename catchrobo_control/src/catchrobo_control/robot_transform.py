#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

### world - robot座標変換
class RobotTransform:
    def __init__(self):
        # tf 
        pass


    #### robot座標系での[m] -> motor回転角度[rad]に変換. gripperは入力をそのまま返す
    def robot2world(self, motor_id, position):
        return


    #### motor回転角度[rad] -> rviz表示用の値に変換　gripperは入力をそのまま返す
    def world2robot(self,motor_id, rad):
        return        

