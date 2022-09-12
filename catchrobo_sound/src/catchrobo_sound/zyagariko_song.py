#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from tkinter.messagebox import RETRY
from catchrobo_sound.my_sound_play import SoundClient
from catchrobo_msgs.msg import MyRosCmd
from catchrobo_manual.manual_command import ManualCommand

from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.jagarico.database import Database
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


class GuiMenu(IntEnum):
    NONE = 0
    ORIGIN = auto()
    CALIBRATION = auto()
    POINT1 = auto()
    POINT2 = auto()
    POINT3 = auto()
    POINT4 = auto()
    INIT = auto()
    START = auto()


class Sound:
    def __init__(self):
        self._running_list = [False, False, False, False]
        self._sound_id = 0
        zyaga_sound = "zyaga.mp3"
        riko_sound = "riko.mp3"

        pi = "pi_.mp3"
        # self._sound_list = [ka]
        # rospy.loginfo(self._sound_list)
        self._sound_list = [pi] + [None] * 100

        # self._sound_list = [
        #     zyaga_sound,
        #     # zyaga_sound,
        #     # riko_sound,
        #     # riko_sound,
        #     # zyaga_sound,
        #     # riko_sound,
        # ]
        self._manual_sound_list = ["goal1.mp3"] + [None] * 100
        self._manual_sound_id = 0

        startup = "1_startup.mp3"
        origin = "2_origin.mp3"
        calib = "3_calib_start.mp3"
        calib_1st = "4_calib_1st_point.mp3"
        calib_2nd = "5_calib_2nd_point.mp3"
        calib_3rd = "6_calib_3rd_point.mp3"
        calib_4th = "7_calib_4th_point.mp3"
        calib_done = "8_calib_done.mp3"
        init = "9_init.mp3"
        start = "10_game_start.mp3"
        done = "11_game_done.mp3"
        self._gui_sound_list = [
            origin,
            None,
            calib,
            calib_1st,
            calib_2nd,
            calib_3rd,
            calib_done,
            init,
            start,
        ]
        self._gui_sound = None
        self._current_sound = None
        self._sound_timer = rospy.Time.now()

        self._is_manual = False
        self._soundhandle = SoundClient()
        rospy.Subscriber("error", ErrorCode, self.error_callback, queue_size=5)
        rospy.Subscriber("ros_cmd", MyRosCmd, self.ros_cmd_callback)
        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber("auto2manual_command", Int8, self.manual_callback)
        rospy.Subscriber("menu", Int8, self.gui_callback)

    #############################################################
    ### callback
    #############################################################
    def error_callback(self, msg: ErrorCode):
        if msg.error_code == ErrorCode.FINISH:
            self._running_list[msg.id] = False

    def ros_cmd_callback(self, msg: MyRosCmd):
        if msg.mode == MyRosCmd.POSITION_CTRL_MODE:
            self._running_list[msg.id] = True

    def manual_callback(self, msg):
        if msg.data == ManualCommand.MANUAL_ON:
            self._is_manual = True
            self._soundhandle.stop()
            self._manual_sound_id = 0
        elif msg.data == ManualCommand.MANUAL_OFF:
            self._is_manual = False

    def gui_callback(self, msg):
        if msg.data == 16:
            self._gui_sound = self._gui_sound_list[GuiMenu.INIT]
            return
        if msg.data > GuiMenu.START or msg.data == GuiMenu.INIT:
            self._gui_sound = None
            return
        self._gui_sound = self._gui_sound_list[msg.data]

    #############################################################
    ### function
    #############################################################

    def gui_alert(self):
        sound = self._gui_sound
        self._gui_sound = None
        return sound

    def manual_alert(self):
        if self._is_manual:
            # rospy.loginfo("manual_alert")
            sound = self._manual_sound_list[self._manual_sound_id]
            self._manual_sound_id += 1
            self._manual_sound_id %= len(self._manual_sound_list)
            return sound
        return None

    def running_alert(self):

        if sum(self._running_list) == 0:
            ##収束したあとは音を鳴らさない
            return None
        # rospy.loginfo("running_alert")

        sound = self._sound_list[self._sound_id]
        self._sound_id += 1
        self._sound_id %= len(self._sound_list)
        return sound

    def decide_sound(self):
        sound = self.gui_alert()
        if sound is not None:
            return sound
        sound = self.manual_alert()
        if sound is not None:
            return sound
        sound = self.running_alert()
        if sound is not None:
            return sound
        return None

    def spin(self):
        rate = rospy.Rate(100)
        old_sound = None
        while not rospy.is_shutdown():
            sound = self.decide_sound()
            # rospy.loginfo(sound)
            if sound is not None:
                self._soundhandle.play(sound)
                self._soundhandle.wait()
                # rospy.sleep(0.5)

                # for i in range(100):
                #     if not rospy.is_shutdown() and sound == self.decide_sound():
                #         rospy.sleep(0.01)
                #     else:
                #         break
            #     if sound != self._current_sound:
            #         self._soundhandle.stop()
            #         self._soundhandle.play(sound)

            # self._current_sound = sound

            #     ### [WARN] 再生が終わる前に再生し出すとコマンドラインに打ち込んだ文字が見えなくなる
            #     self._soundhandle.stop()

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("zyagariko_song", anonymous=True)
    node = Sound()
    node.spin()
