#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

### world - robot座標変換
class WorldRobotTransform:
    def __init__(self, field):
        # world座標でみたrobot座標原点
        self._robot_origin = np.array([0.09045, 0.845, 0.0705, 0])
        if field == "blue":
            self._robot_origin[1] *= -1

    #### robot座標系での[m] -> world[m]に変換.
    def robot2world(self, position):
        return np.array(position[:3]) + self._robot_origin

    #### world座標系での[m] -> robot [m]
    def world2robot(self, position):
        print(position, self._robot_origin)
        temp = np.array(position[:3]) - self._robot_origin
        print(temp)
        return temp

    def world2robot_each(self, id, position):
        return position - self._robot_origin[id]
