#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from catchrobo_manager.game_manager import GameManager


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # while not rospy.is_shutdown():
    ### 初期化
    rospy.sleep(0.5)
    game_manager.init()
    ### main動作
    game_manager.spin()
