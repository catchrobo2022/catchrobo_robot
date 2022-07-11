#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray


class GuiBridge:
    def __init__(self, gui_input_ros_output, ros_input_gui_output, init_data):
        self._pub2gui = rospy.Publisher(
            gui_input_ros_output, Int32MultiArray, queue_size=1
        )
        rospy.Subscriber(ros_input_gui_output, Int32MultiArray, self.guiCallback)
        self._msg = Int32MultiArray()
        self._msg.data = list(init_data)

    def guiCallback(self, msg):
        self._msg = msg

    def getMsg(self):
        return self._msg

    def sendGUI(self, data):
        self._msg.data = list(data)
        self._pub2gui.publish(self._msg)
