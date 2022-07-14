#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.ros_cmd_template import RosCmdTemplate

import rospy
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/ros_cmd", MyRosCmd, queue_size=1)
    pub_enable = rospy.Publisher("/enable_cmd", EnableCmd, queue_size=1)
    rospy.sleep(1)  # rosが起動するのを待つ

    ### default command生成器
    template = RosCmdTemplate()

    ################################################### motor on 指示
    enable_command = template.generate_enable_command(True, False)
    pub_enable.publish(enable_command)
    print(enable_command)
    rospy.sleep(1)

    ################################################# joint command
    ### これで全要素それっぽい値が入ったcommandを作成できる
    ### robot_position : 目標位置[m], robot_end_velocity : 終端速度[v/s]
    command = template.generate_ros_command(
        id=2,
        mode=MyRosCmd.POSITION_CTRL_MODE,
        robot_position=0,
        robot_end_velocity=0,
    )
    print(command)
    pub.publish(command)

    command = template.generate_ros_command(
        id=1,
        mode=MyRosCmd.POSITION_CTRL_MODE,
        robot_position=-1,
        robot_end_velocity=0,
    )
    print(command)
    pub.publish(command)
    rospy.sleep(10)

    command = template.generate_ros_command(
        id=1,
        mode=MyRosCmd.POSITION_CTRL_MODE,
        robot_position=1,
        robot_end_velocity=0,
    )
    print(command)
    pub.publish(command)
    rospy.sleep(1)

    # command = template.generate_ros_command(
    #     id=0,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0,
    #     robot_end_velocity=0,
    # )
    # print(command)
    # pub.publish(command)
    # rospy.sleep(2)

    # command = template.generate_ros_command(
    #     id=0,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.1,
    #     robot_end_velocity=0,
    # )
    # print(command)
    # pub.publish(command)
    # rospy.sleep(10)

    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.0,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)
    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=1.0,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)
    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=1.0,
    #     robot_end_velocity=0.0,
    # )
    # # command.acceleration_limit = template.robot_m2rad(command.id, 0.5)
    # print(command)
    # pub.publish(command)
    # command.kp = 5
    # command.kd = 0.5
    ### 生成したcommandはあくまでdefault. 全要素自由に書き換え可能
    ### robot_m2rad : ロボット座標系でのmをradに変換する
    # command.velocity_limit = template.robot_m2rad(command.id, 0.5)

    #######################################
    # command.id = 1  # 0: x軸 1:y軸 2: z軸 3:グリッパー
    # command.mode = (
    #     MyRosCmd.POSITION_CTRL_MODE
    # )  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.DIRECT_CTRL_MODE
    # command.position = rad_transform.robot_m2rad(command.id, 1.0)
    # command.velocity = rad_transform.robot_m2rad(command.id, 0.0)  # 目標位置での速度
    # command.net_inertia = 0.0  # 慣性モーメント(未対応)
    # command.effort = 0  # 自重補償項(未対応)
    # command.position_min = rad_transform.robot_m2rad(command.id, 0.0)  # 可動域
    # command.position_max = rad_transform.robot_m2rad(command.id, 1.5)  # 可動域
    # command.velocity_limit = rad_transform.robot_m2rad(command.id, 1.0)  # 台形加速中の最大速度
    # command.acceleration_limit = rad_transform.robot_m2rad(
    #     command.id, 2.0
    # )  # 台形加速を作るための最大加速度
    # command.jerk_limit = rad_transform.robot_m2rad(
    #     command.id, 5.0
    # )  # 台形加速を作るための最大躍度(加速度の微分)
    # command.kp = 5  # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
    # command.kd = 0.5

    # rospy.sleep(1) # enableを待つ
    ### robot_position : 目標位置[m], robot_end_velocity : 終端速度[v/s]
    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.13,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)

    # command = template.generate_ros_command(
    #     id=1,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.0,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)
    # command = template.generate_ros_command(
    #     id=1,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.0,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)
    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.0,
    #     robot_end_velocity=0.0,
    # )
    # print(command)
    # pub.publish(command)
    # command = template.generate_ros_command(
    #     id=1,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.3,
    #     robot_end_velocity=0.0,
    # )
    # # command.acceleration_limit = template.robot_m2rad(command.id, 0.5)
    # print(command)
    # pub.publish(command)

    # ### 原点に戻る
    # command.mode = MyRosCmd.POSITION_CTRL_MODE
    # command.position = 0
    # print(command)
    # pub.publish(command)

    # enable_command.is_enable = False
    # pub_enable.publish(enable_command)
    # print(enable_command)
    # rospy.sleep(2)
