#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_sound.my_sound_play import SoundClient
from catchrobo_msgs.msg import MyRosCmd

from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.jagarico.database import Database
from catchrobo_manual.manual_command import ManualCommand
import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from catchrobo_msgs.msg import ErrorCode

import numpy as np
import queue

from enum import IntEnum, auto

import subprocess
from datetime import datetime
import os


class SoundEnum(IntEnum):
    ARRIVE_X = auto()
    ARRIVE_Y = auto()
    ARRIVE_Z = auto()
    GRIPPER = auto()


class Sound:
    def __init__(self):
        self._running_list = [False, False, False, False]
        self._sound_id = 0
        zyaga_sound = "zyaga.mp3"
        riko_sound = "riko.mp3"
        self._sound_list = [
            zyaga_sound,
            zyaga_sound,
            riko_sound,
            riko_sound,
            zyaga_sound,
            riko_sound,
        ]
        # self._sound_list = ["get.mp3"]

        self._soundhandle = SoundClient()
        rospy.Subscriber("error", ErrorCode, self.error_callback, queue_size=5)
        rospy.Subscriber("ros_cmd", MyRosCmd, self.ros_cmd_callback)

    #############################################################
    ### callback
    #############################################################
    def error_callback(self, msg: ErrorCode):
        if msg.error_code == ErrorCode.FINISH:
            self._running_list[msg.id] = False

    def ros_cmd_callback(self, msg: MyRosCmd):
        if msg.mode == MyRosCmd.POSITION_CTRL_MODE:
            self._running_list[msg.id] = True

    #############################################################
    ### function
    #############################################################

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if sum(self._running_list) == 0:
                ##収束したあとは音を鳴らさない
                rate.sleep()
                continue

            sound = self._sound_list[self._sound_id]
            self._sound_id += 1
            self._sound_id %= len(self._sound_list)
            self._soundhandle.play(sound)
            self._soundhandle.wait()
            # rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("zyagariko_song", anonymous=True)
    node = Sound()
    node.spin()
