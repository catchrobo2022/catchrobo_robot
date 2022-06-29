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
    enable_command.enable_check = False
    rospy.sleep(1)  # rosが起動するのを待つ
    pub_enable.publish(enable_command)
    print(enable_command)
    rospy.sleep(1)

    command = template.generate_origin_command(0, 0.05)
    command.acceleration_limit = 1
    command.kd = 0.5
    print(command)
    pub.publish(command)
    rospy.sleep(10)
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
    # rospy.sleep(10)  # このプログラムは3秒後に自動終了する. このsleepが無いとpublish前に終了してしまう

    enable_command.is_enable = False
    pub_enable.publish(enable_command)
    print(enable_command)
    rospy.sleep(2)
