#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.robot import Robot
from catchrobo_manager.jagarico.database import Database

import rospy

if __name__ == "__main__":
    rospy.init_node("test_pub")
    robot = Robot("blue")
    rospy.sleep(1)  # rosが起動するのを待つ
    robot.enable()
    # rospy.sleep(2)
    robot.control_permission(True)

    database = Database()
    csv_name = "blue" + "_shoot.csv"
    database.readCsv(csv_name)

    # self.BOX_IDS = [0, 2, 15, 17]
    target = database.getPosi(0)
    robot.go(x=target[0] + 0.05, y=target[1] + 0.05, z=target[2])
    ### main動作
    # game_manager.spin()
