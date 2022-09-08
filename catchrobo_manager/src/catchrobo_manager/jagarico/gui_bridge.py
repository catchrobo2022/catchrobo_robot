#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray, Int8


class GuiBridge:
    def __init__(
        self,
        gui_input_ros_output,
        ros_input_gui_output,
        target_topic,
        init_data,
        callback_function,
    ):
        self._pub2gui = rospy.Publisher(
            gui_input_ros_output, Int32MultiArray, queue_size=1
        )
        self._pub_target = rospy.Publisher(target_topic, Int8, queue_size=1)
        rospy.Subscriber(ros_input_gui_output, Int32MultiArray, callback_function)
        self._msg = Int32MultiArray()
        self._msg.data = list(init_data)

    # def guiCallback(self, msg):
    #     self._msg = msg

    def getMsg(self):
        return self._msg

    def sendGUI(self, data):
        self._msg.data = list(data)
        self._pub2gui.publish(self._msg)

    def send_target(self, id):
        data = Int8(id)
        self._pub_target.publish(data)
