#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_control.catchrobo_center import CatchroboCenter

import rospy
from std_msgs.msg import Int8


class ManualCommand:
    NONE = 0
    START = 1
    ENABLE = 2
    DISABLE = 3
    STOP = 4


class GameManager():
    def __init__(self):
        self._catchrobo_center = CatchroboCenter()
        rospy.Subscriber("manual", Int8, self.manualCallback)
    
    def manualCallback(self, msg):
        if(msg.data== ManualCommand.START):
            self._catchrobo_center.enable()
            self._catchrobo_center.main()
        elif(msg.data== ManualCommand.ENABLE):
            self._catchrobo_center.enable()
        elif(msg.data== ManualCommand.DISABLE):
            self._catchrobo_center.disable()
        elif(msg.data == ManualCommand.STOP):
            self._catchrobo_center.stop()      

    def init(self):
        pass

        

    
if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    game_manager.init()
    rospy.spin()