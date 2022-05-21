#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")

    pub = rospy.Publisher("/my_joint_control", MyRosCmdArray, queue_size=1)

    command = MyRosCmdArray()
    motor = [MyRosCmd(), MyRosCmd(), MyRosCmd()]

    for i in range(3):
        motor[i].id = i
        motor[i].mode = MyRosCmd.VELOCITY_CTRL_MODE

    motor[0].mode = MyRosCmd.POSITION_CTRL_MODE
    motor[0].position = 1.5
    motor[0].velocity = 0
    motor[0].inertia = 1.0
    motor[0].effort = 0
    motor[0].velocity_limit = 0.5
    motor[0].acceleration_limit = 0.3
    motor[0].jerk_limit = 0.1
    motor[0].kp = 0  #1
    motor[0].kd = 1 #0


    command.command_array = motor

    print(command)
    rospy.sleep(1)
    pub.publish(command)
    rospy.sleep(3)

