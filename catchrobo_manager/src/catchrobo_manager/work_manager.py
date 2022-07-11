#!/usr/bin/env python
# -*- coding: utf-8 -*-


from catchrobo_manager.next_action_enum import NextAction
from catchrobo_manager.jagarico.database import Database
from jagarico.target_jagarico_calculator import TargetJagaricoCalculator


class WorkManager:
    def __init__(self, field):
        # [TODO] targetをGUIに教える

        self._database = Database()
        csv_name = field + "_jagarico.csv"
        self._database.readCsv(csv_name)
        self._calculator = TargetJagaricoCalculator()

    def get_target_id(self):
        target_id = self._calculator.calcTarget(self._database)
        return target_id

    def get_target_posi(self):
        target_id = self._calculator.calcTarget(self._database)
        position = self._database.getPosi(target_id)

        # 今だけmm単位からmに変換
        for i in range(len(position)):
            position[i] *= 0.001
        return position

    def pick(self):
        pick_id = self.get_target_id()
        self._database.delete(pick_id)

        ## [TODO] 次動作計算アルゴリズム
        ## 次も連続してじゃがりこを回収するなら
        next_action = NextAction.PICK
        ## もうシュートするなら
        next_action = NextAction.SHOOT
        return next_action

    def setCanGoCommon(self, flag):
        self._calculator.setCanGoCommon(flag)
