#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.ros_cmd_template import RosCmdTemplate

import rospy
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/ros_cmd", MyRosCmd, queue_size=1)
    pub_enable = rospy.Publisher("/enable_cmd", EnableCmd, queue_size=1)
    rospy.sleep(1)  # rosが起動するのを待つ

    ### default command生成器
    template = RosCmdTemplate()
    template.set_accerelation_limit_scale(0.5)

    ################################################### motor on 指示
    enable_command = template.generate_enable_command(True, True)
    # enable_command.enable_check = False
    pub_enable.publish(enable_command)
    print(enable_command)
    rospy.sleep(1)

    ################################################# joint command
    ### これで全要素それっぽい値が入ったcommandを作成できる
    ### robot_position : 目標位置[m], robot_end_velocity : 終端速度[v/s]

    command = template.generate_velocity_command(
        id=2,
        velocity_m=0,
    )
    command.kp = 0
    command.kd = 0
    print(command)
    pub.publish(command)
    rospy.sleep(3)

    # command = template.generate_ros_command(
    #     id=2,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.12,
    #     robot_end_velocity=0,
    # )
    # print(command)
    # pub.publish(command)
    # rospy.sleep(5)

    # command = template.generate_ros_command(
    #     id=1,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=0.6,
    #     robot_end_velocity=0,
    # )
    # print(command)
    # pub.publish(command)
    # rospy.sleep(5)

    # command = template.generate_ros_command(
    #     id=1,
    #     mode=MyRosCmd.POSITION_CTRL_MODE,
    #     robot_position=-0.6,
    #     robot_end_velocity=0,
    # )
    # print(command)
    # pub.publish(command)
    # rospy.sleep(10)

    command = template.generate_ros_command(
        id=0,
        mode=MyRosCmd.POSITION_CTRL_MODE,
        robot_position=1,
        robot_end_velocity=0,
    )
    print(command)
    pub.publish(command)
    rospy.sleep(10)

    enable_command.is_enable = False
    pub_enable.publish(enable_command)
    print(enable_command)
    rospy.sleep(2)
