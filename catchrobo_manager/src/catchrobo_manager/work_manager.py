#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Point

from catchrobo_manager.next_action_enum import NextAction
from catchrobo_manager.jagarico.jagarico_database import JagaricoDatabase
from jagarico.target_jagarico_calculator import TargetJagaricoCalculator

class WorkManager:
    def __init__(self, field):
        # [TODO] csv読み込み　
        # targetをGUIに教える

        self._database = JagaricoDatabase()
        self._database.readCsv(field)
        self._calculator=TargetJagaricoCalculator()

    def get_target_id(self):
        target_id = self._calculator.calcTarget(self._database)
        return target_id
    
    def get_target_posi(self):
        ### [TODO] 目標ビスコ位置計算
        target_id = self._calculator.calcTarget(self._database)
        # geometry_msgのPoint型でpositionを渡す（去年のがそうなってた）
        position = self._database.getPosi(target_id)
        ### [TODO] next_action計算

        return position

    def pick(self):
        ### [TODO] じゃがりこ取得
        pick_id=self.get_target_id()
        self._database.delete(pick_id)
        
        next_action = NextAction.PICK
        return next_action

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

    #################################去年のプログラム
    def pick(self, id):
        self._rviz.attach(id)
        self._database.delete(id)
        self._gui.sendGUI()

    def release(self, id):
        self._rviz.release(id)

    def calcTargetTwin(self):
        self._target_ids = self._calculator.calcTargetTwin(self._database)
        self._twin = self._calculator.isNeighbor(
            self._database, self._target_ids[0], self._target_ids[1]
        )

        self._gui.highlight(self._target_ids)

    def getTargetTwin(self):
        return [self._database.getObj(id) for id in self._target_ids], self._twin

    def setCanGoCommon(self, flag):
        self._can_go_common = flag
        self._calculator.setCanGoCommon(flag)

    def isCommonExist(self):
        return self._database.isCommonExist()
