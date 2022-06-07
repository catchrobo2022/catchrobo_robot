#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd
import math
import copy


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)

    N_MOTOR = 1

    command = MyRosCmd()

    command.id = 0 # 0: x軸 1:y軸 2: z軸 3:グリッパー
    command.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.DIRECT_CTRL_MODE
    command.position = 0 #3*math.pi 
    command.velocity = -math.pi/10 * 0#目標位置での速度
    command.mass = 0.0 # 慣性モーメント(未対応)
    command.effort = 0 #自重補償項(未対応)
    command.position_min = -60 #可動域
    command.position_max = 60 #可動域
    command.velocity_limit = 30 #台形加速中の最大速度
    command.acceleration_limit = 10 #台形加速を作るための最大加速度
    command.jerk_limit = 5 #台形加速を作るための最大躍度(加速度の微分)
    command.kp = 0 # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    command.kd = 1


    
    command = command


    command_array = MyRosCmdArray()
    command_array.command_array.append(command)

    rospy.sleep(1) # rosが起動するのを待つ
    print(command_array)
    pub.publish(command_array)
    rospy.sleep(10) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう


    for i in range(N_MOTOR):
        command_array.command_array[i].kp = 0
        command_array.command_array[i].kd = 0
    print(command_array)
    pub.publish(command_array)
    rospy.sleep(3) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう




