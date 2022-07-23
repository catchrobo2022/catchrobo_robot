#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy


class Calibration:
    def gui_callback(self):
        ### [TODO] 各辺の測定開始、終了を受け取る

        ### [TODO] 4辺の情報が集まったらtfを更新

        pass

    def robot_position_callback(self):
        ### [TODO] 現在値の取得

        pass


if __name__ == "__main__":
    rospy.init_node("calibration")
    calibration = Calibration()
    rospy.spin()
