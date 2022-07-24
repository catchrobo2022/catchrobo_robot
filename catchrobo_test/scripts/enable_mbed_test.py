#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.ros_cmd_template import RosCmdTemplate

import rospy
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/ros_cmd", MyRosCmd, queue_size=1)
    pub_enable = rospy.Publisher("/enable_cmd", EnableCmd, queue_size=1)

    ### default command生成器
    template = RosCmdTemplate()

    ################################################### motor on 指示
    enable_command = template.generate_enable_command()
    enable_command.enable_check = True
    rospy.sleep(1)  # rosが起動するのを待つ
    pub_enable.publish(enable_command)
    print(enable_command)
    # ################################################# joint command
    # ### enable内に戻る
    # command = template.generate_velocity_command(0, 10, 0)
    # command.position_max = template.robot_m2rad(command.id, 10000)
    # command.velocity_limit = template.robot_m2rad(command.id, 15)
    # print(command)
    # pub.publish(command)
    # rospy.sleep(1)

    # enable_command = template.generate_enable_command()
    # enable_command.enable_check = True
    # pub_enable.publish(enable_command)
    # print(enable_command)
    # rospy.sleep(10)

    # ### enableを破るようposition_limitを外す
    # # command.position_min = template.robot_m2rad(command.id, -5)
    # ### 生成したcommandはあくまでdefault. 全要素自由に書き換え可能
    # ### robot_m2rad : ロボット座標系でのmをradに変換する
    # # command.velocity_limit = template.robot_m2rad(command.id, 0.5)

    # #######################################
    # # command.id = 1  # 0: x軸 1:y軸 2: z軸 3:グリッパー
    # # command.mode = (
    # #     MyRosCmd.POSITION_CTRL_MODE
    # # )  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.VELOCITY_CTRL_MODE
    # # command.position = rad_transform.robot_m2rad(command.id, 1.0)
    # # command.velocity = rad_transform.robot_m2rad(command.id, 0.0)  # 目標位置での速度
    # # command.net_inertia = 0.0  # 慣性モーメント(未対応)
    # # command.effort = 0  # 自重補償項(未対応)
    # # command.position_min = rad_transform.robot_m2rad(command.id, 0.0)  # 可動域
    # # command.position_max = rad_transform.robot_m2rad(command.id, 1.5)  # 可動域
    # # command.velocity_limit = rad_transform.robot_m2rad(command.id, 1.0)  # 台形加速中の最大速度
    # # command.acceleration_limit = rad_transform.robot_m2rad(
    # #     command.id, 2.0
    # # )  # 台形加速を作るための最大加速度
    # # command.jerk_limit = rad_transform.robot_m2rad(
    # #     command.id, 5.0
    # # )  # 台形加速を作るための最大躍度(加速度の微分)
    # # command.kp = 5  # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    # # command.kd = 0.5

    # # rospy.sleep(1) # enableを待つ

    # ### 原点に戻る
    # enable_command = template.generate_enable_command()
    # enable_command.enable_check = False
    # pub_enable.publish(enable_command)
    # print(enable_command)
    # rospy.sleep(1)  # rosが起動するのを待つ

    # command = template.generate_ros_command(0, MyRosCmd.POSITION_CTRL_MODE, 0)
    # print(command)
    # pub.publish(command)
    # rospy.sleep(10)  # このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう

    # enable_command.is_enable = False
    # pub_enable.publish(enable_command)
    # print(enable_command)
    rospy.sleep(2)
