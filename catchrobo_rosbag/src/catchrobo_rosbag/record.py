#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.gui_menu_enum import GuiMenu


import rospy
from std_msgs.msg import Int8

import subprocess


class Record:
    def __init__(self):
        self._is_recording = False
        rospy.Subscriber("menu", Int8, self.gui_callback, queue_size=1)

    #############################################################
    ### callback
    #############################################################
    def gui_callback(self, msg) -> None:
        menu = msg.data
        if menu == GuiMenu.START:
            self.start_recording()

    #############################################################
    ### function
    #############################################################
    def start_recording(self):
        if self._is_recording is True:
            return
        self._is_recording = True
        subprocess.run(["roslaunch", "catchrobo_rosbag", "rosbag_record.launch"])


if __name__ == "__main__":
    rospy.init_node("record_starter", anonymous=True)
    node = Record()
    rospy.spin()
