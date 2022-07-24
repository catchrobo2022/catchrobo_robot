#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_driver.ros_cmd_template import RosCmdTemplate

import rospy
from catchrobo_msgs.msg import MyRosCmdArray, MyRosCmd, EnableCmd


if __name__ == "__main__":
    rospy.init_node("test_pub")
    pub = rospy.Publisher("/ros_cmd", MyRosCmd, queue_size=1)
    field = rospy.get_param("/field")
    rospy.sleep(1)  # rosが起動するのを待つ

    template = RosCmdTemplate()
    for i in range(3):
        command = template.generate_origin_command(i, field)
        print(command)
        pub.publish(command)
        rospy.sleep(0.5)
