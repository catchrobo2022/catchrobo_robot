#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cgitb import enable
from sqlite3 import enable_callback_tracebacks
from catchrobo_driver.rad_transform import RadTransform

import rospy
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd
import math
import copy


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)
    pub_enable = rospy.Publisher("/enable_cmd", EnableCmd, queue_size=1)

    N_MOTOR = 1

    rad_transform = RadTransform()

    ### 本当に危険なとき用の制約を設定
    enable_command = EnableCmd()
    ### motor電源を入れる
    enable_command.is_enable = True
    ### x, y, zの順
    ### 可動域
    enable_command.position_min = [0.0, rad_transform.robot_m2rad(1, -1.5), 0]
    enable_command.position_max = [
        rad_transform.robot_m2rad(0, 1.5),
        rad_transform.robot_m2rad(1, 1.5),
        rad_transform.robot_m2rad(2, 1.5),
    ]
    ### 速度制約
    enable_command.velocity_limit = [rad_transform.robot_m2rad(i, 15) for i in range(3)]
    ### トルク制約
    enable_command.torque_limit = [rad_transform.robot_m2rad(i, 30) for i in range(3)]
    ### 目標位置とどれくらいかけ離れていたらだめか
    enable_command.trajectory_error_limit = [
        rad_transform.robot_m2rad(i, 0.5) for i in range(3)
    ]

    ### 障害物としての箱を設定。 x,y,zの値
    enable_command.obstacle_min = [1] * 3  # min > maxにすると、制約無しとなる
    enable_command.obstacle_max = [-1] * 3
    # enable_command.obstacle_min = [enable_command.position_min[0], rad_transform.robot_m2rad(1, 0.5), rad_transform.robot_m2rad(2, 0)]
    # enable_command.obstacle_max = [enable_command.position_max[0], rad_transform.robot_m2rad(1, 1.5), rad_transform.robot_m2rad(2, 0.2)]

    rospy.sleep(1)  # rosが起動するのを待つ
    pub_enable.publish(enable_command)

    ###########################################################
    command = MyRosCmd()

    command.id = 1  # 0: x軸 1:y軸 2: z軸 3:グリッパー
    command.mode = (
        MyRosCmd.POSITION_CTRL_MODE
    )  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.DIRECT_CTRL_MODE
    command.position = rad_transform.robot_m2rad(command.id, 1.0)
    command.velocity = rad_transform.robot_m2rad(command.id, 0.0)  # 目標位置での速度
    command.net_inertia = 0.0  # 慣性モーメント(未対応)
    command.effort = 0  # 自重補償項(未対応)
    command.position_min = rad_transform.robot_m2rad(command.id, 0.0)  # 可動域
    command.position_max = rad_transform.robot_m2rad(command.id, 1.5)  # 可動域
    command.velocity_limit = rad_transform.robot_m2rad(command.id, 1.0)  # 台形加速中の最大速度
    command.acceleration_limit = rad_transform.robot_m2rad(
        command.id, 2.0
    )  # 台形加速を作るための最大加速度
    command.jerk_limit = rad_transform.robot_m2rad(
        command.id, 5.0
    )  # 台形加速を作るための最大躍度(加速度の微分)
    command.kp = 5  # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    command.kd = 0.5

    command = command

    command_array = MyRosCmdArray()
    command_array.command_array.append(command)

    # rospy.sleep(1) # enableを待つ
    print(command_array)
    pub.publish(command_array)
    rospy.sleep(10)  # このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう

    for i in range(N_MOTOR):
        command_array.command_array[i].position = 0
        # command_array.command_array[i].kd = 0
    print(command_array)
    pub.publish(command_array)
    rospy.sleep(3)  # このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう
