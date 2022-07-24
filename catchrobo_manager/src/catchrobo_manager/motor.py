#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_driver.ros_cmd_template import RosCmdTemplate

import rospy
from std_msgs.msg import Int8
from catchrobo_msgs.msg import (
    ErrorCode,
    EnableCmd,
    MyRosCmdArray,
    MyRosCmd,
    PegInHoleCmd,
)
from sensor_msgs.msg import JointState

### input : ロボット座標系 [m]
class Motor:
    def __init__(self, id, ros_cmd_template):
        self._running = False
        self._id = id
        self._ros_cmd_template = ros_cmd_template

        self._pub_ros_cmd = rospy.Publisher("ros_cmd", MyRosCmd, queue_size=5)

    def finish(self):
        self._running = False

    def is_running(self):
        return self._running

    def go(self, target_position, has_work_num=0):
        self._running = True
        ros_command = self._ros_cmd_template.generate_ros_command(
            self._id, MyRosCmd.POSITION_CTRL_MODE, target_position, 0, has_work_num
        )
        self._pub_ros_cmd.publish(ros_command)
        print(ros_command)

    def direct_control(self, position, velocity, has_work_num):
        self._running = True
        ros_command = self._ros_cmd_template.generate_ros_command(
            self._id, MyRosCmd.VELOCITY_CTRL_MODE, position, velocity, has_work_num
        )
        self._pub_ros_cmd.publish(ros_command)

    def set_origin(self, field):
        ros_command = self._ros_cmd_template.generate_origin_command(self._id, field)
        print(ros_command)
        self._pub_ros_cmd.publish(ros_command)
