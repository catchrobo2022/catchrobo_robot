#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)


    command = MyRosCmd()

    command.id = 0    # 0: x軸 1:y軸 2: z軸 3:グリッパー
    command.mode = MyRosCmd.VELOCITY_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
    command.position = 0
    command.velocity = 0
    command.inertia = 1.0
    command.effort = 0
    command.position_min = 0
    command.position_max = 1.5
    command.velocity_limit = 0.5
    command.acceleration_limit = 0.3
    command.jerk_limit = 0.1
    command.kp = 0
    command.kd = 1 

    command_array = MyRosCmdArray()
    command_array.command_array = [command] # command1,2,...を同様に作成し、[command, command1, ...]と同時に送ることも可能

    print(command_array)
    rospy.sleep(1) # rosが起動するのを待つ
    pub.publish(command_array)
    rospy.sleep(3) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう

