#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd
import math
import copy


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)


    command = MyRosCmd()

    command.id = 0 # 0: x軸 1:y軸 2: z軸 3:グリッパー
    command.mode = MyRosCmd.VELOCITY_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
    command.position =3*math.pi
    command.velocity = math.pi#目標位置での速度
    command.mass = 0.0 # 慣性モーメント(未対応)
    command.effort = 0 #自重補償項(未対応)
    command.position_min = -60 #可動域
    command.position_max = 60 #可動域
    command.velocity_limit = 30 #台形加速中の最大速度
    command.acceleration_limit = 10 #台形加速を作るための最大加速度
    command.jerk_limit = 5 #台形加速を作るための最大躍度(加速度の微分)
    command.kp = 0 # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    command.kd = 0.5


    
    command = command


    command_array = MyRosCmdArray()

    for i in range(3):
        temp = copy.deepcopy(command)
        temp.id = i
        temp.position = (i+1) * 2*math.pi 
        command_array.command_array.append(temp)


    print(command_array)
    rospy.sleep(1) # rosが起動するのを待つ
    pub.publish(command_array)
    rospy.sleep(10) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう


    for i in range(3):
        command_array.command_array[i].kp = 0
        command_array.command_array[i].kd = 0
    pub.publish(command_array)
    rospy.sleep(3) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう




