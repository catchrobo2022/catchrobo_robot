#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import Int8

import jtalk

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

jtalk.gtalk(startup)

mode = [0]


def callback(msg):
    mode[0] = msg.data
    if mode[0] == 0:
        jtalk.gtalk(origin)
    # if mode[0] == 1:
    #    jtalk.gtalk(origin)
    elif mode[0] == 2:
        jtalk.gtalk(calib)
    elif mode[0] == 3:
        jtalk.gtalk(calib_1st)
    elif mode[0] == 4:
        jtalk.gtalk(calib_2nd)
    elif mode[0] == 5:
        jtalk.gtalk(calib_3rd)
    elif mode[0] == 6:
        jtalk.gtalk(calib_4th)
    elif mode[0] == 16:
        jtalk.gtalk(init)
    elif mode[0] == 8:
        jtalk.gtalk(start)


if __name__ == "__main__":
    rospy.init_node("gui_voice")
    sub = rospy.Subscriber("menu", Int8, callback)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
