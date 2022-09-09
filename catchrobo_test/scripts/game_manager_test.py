#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.game_manager import GameManager

import rospy

if __name__ == "__main__":
    rospy.init_node("GameManager")
    # rospy.sleep(0.1)
    game_manager = GameManager()
    # while not rospy.is_shutdown():
    ### 初期化
    rospy.set_param("ros_cmd/acceleration_limit_scale", 0.1)
    game_manager.init()
    ### main動作
    game_manager.spin()
