#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Int32MultiArray

class BiscoGUI():
    def __init__(self, database):
        self._database = database
        
        self._pub2gui = rospy.Publisher("ob", Int32MultiArray, queue_size=1)
        rospy.Subscriber("obj", Int32MultiArray, self.guiCallback)
        self._pub_highlight = rospy.Publisher("/highlight", Int32MultiArray, queue_size=1)

        
    ###[TODO] 前日
    def setObstacle(self, obstacle):
        self._obstacle = obstacle

    def highlight(self, target_ids):
        if target_ids[0] is not None:
            highlight_id = Int32MultiArray()
            if target_ids[1] is not None:
                highlight_id.data  = target_ids
            else:
                highlight_id.data  = target_ids[0:1]
            self._pub_highlight.publish(highlight_id)

    def sendGUI(self):
        info = Int32MultiArray()
        bisco_num = self._database.getNum()
        info.data = [0] * bisco_num 
        for i in range(bisco_num):
            info.data[i] = self._database.isExist(i)
        self._pub2gui.publish(info)

    def guiCallback(self,msg):
        # rospy.loginfo(msg.data)
        for i, val in enumerate(msg.data):
            temp = bool(val)
            before = self._database.getState(i, "exist")

            if temp != before:
                rospy.loginfo("change bisco {} -> {}".format(i, temp))
                if temp == 0:
                    self._obstacle.release(i)
            self._database.updateState(i,"exist", temp)

            
