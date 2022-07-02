#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import Int32MultiArray

from catchrobo_manager.bisco.bisco_database import BiscoDatabase
from catchrobo_manager.bisco.bisco_rviz import BiscoRviz
from catchrobo_manager.bisco.bisco_gui import BiscoGUI
from catchrobo_manager.bisco.target_bisco_calculator import TargetBiscoCalculator
import rospy

class BiscoManager():
    def __init__(self, color, rviz= True):
        self._database = BiscoDatabase()
        self._database.readCsv(color)

        self._calculator = TargetBiscoCalculator()
        self._gui = BiscoGUI(self._database)
        self._gui.sendGUI()
        if rviz:
            self._rviz = BiscoRviz()
            self._rviz.addBox2Scene(self._database)

            self._gui.setObstacle(self._rviz)

        self._can_go_common = False

        

    def pick(self, id):
        self._rviz.attach(id)
        self._database.delete(id)
        self._gui.sendGUI()

    def release(self, id):
        self._rviz.release(id)

    def calcTargetTwin(self):
        self._target_ids =  self._calculator.calcTargetTwin(self._database)
        self._twin = self._calculator.isNeighbor(self._database, self._target_ids[0], self._target_ids[1])

        self._gui.highlight(self._target_ids)



    def getTargetTwin(self):
        return [self._database.getObj(id) for id in self._target_ids], self._twin 


    def setCanGoCommon(self, flag):
        self._can_go_common = flag
        self._calculator.setCanGoCommon(flag)

    def isCommonExist(self):
        return self._database.isCommonExist()