#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)


    command = MyRosCmd()

    command.id = 3 # 0: x軸 1:y軸 2: z軸 3:グリッパー
    command.mode = MyRosCmd.POSITION_CTRL_MODE  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
    command.position = 1
    command.velocity = 0 #目標位置での速度
    command.inertia = 1.0 # 慣性モーメント(未対応)
    command.effort = 0 #自重補償項(未対応)
    command.position_min = 0 #可動域
    command.position_max = 1.2 #可動域
    command.velocity_limit = 0.5 #台形加速中の最大速度
    command.acceleration_limit = 0.3 #台形加速を作るための最大加速度
    command.jerk_limit = 0.1 #台形加速を作るための最大躍度(加速度の微分)
    command.kp = 1 # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    command.kd = 0 

    command_array = MyRosCmdArray()
    command_array.command_array = [command] # command1,2,...を同様に作成し、[command, command1, ...]と同時に送ることも可能

    print(command_array)
    rospy.sleep(1) # rosが起動するのを待つ
    pub.publish(command_array)
    rospy.sleep(3) #このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう

