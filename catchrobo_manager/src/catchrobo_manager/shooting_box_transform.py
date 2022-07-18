#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np


class ShootingBoxTransform:
    def __init__(self, shooting_box_center_raw):
        self._shooting_box_center_raw_np = np.array(shooting_box_center_raw)

    def get_calibrated_position(self, position_raw):
        ### position_rawとシューティングボックス中央からの相対座標を出す
        position_raw_np = np.array(position_raw)
        position_on_shooting_box_frame = (
            position_raw_np - self._shooting_box_center_raw_np
        )

        ### [TODO] shooting_box フレームを用いて、一時的なtfを作成

        ### [TODO] world座標にlookup transform

        ### [TODO] world座標を[x,y,z]のlistにして送信
        return position_raw
