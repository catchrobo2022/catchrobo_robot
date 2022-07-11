#!/usr/bin/env python
# -*- coding: utf-8 -*-


from catchrobo_manager.next_action_enum import NextAction
from catchrobo_manager.jagarico.database import Database
from jagarico.target_jagarico_calculator import TargetJagaricoCalculator
from catchrobo_manager.jagarico.gui_bridge import GuiBridge

import rospy


class WorkManager:
    def __init__(self, field):
        # [TODO] targetをGUIに教える

        self._database = Database()
        csv_name = field + "_jagarico.csv"
        self._database.readCsv(csv_name)
        self._calculator = TargetJagaricoCalculator()
        self.EXIST_KEY = "exist"
        msg_template = [1] * self._database.getIdNum()
        self._gui = GuiBridge("obj_giro", "obj_rigo", msg_template)

    def get_target_id(self):
        self.update_by_gui()
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
        self._gui.sendGUI(self._database.getColumn(self.EXIST_KEY))

        ## [TODO] 次動作計算アルゴリズム
        ## 次も連続してじゃがりこを回収するなら
        next_action = NextAction.PICK
        ## もうシュートするなら
        next_action = NextAction.SHOOT
        return next_action

    def setCanGoCommon(self, flag):
        self._calculator.setCanGoCommon(flag)

    def update_by_gui(self):
        msg = self._gui.getMsg()
        for i, val in enumerate(msg.data):
            # rospy.loginfo("i, val {}{}".format(i,val))
            self._database.updateState(i, self.EXIST_KEY, bool(val))


if __name__ == "__main__":
    import rospy

    rospy.init_node("test")
    manager = WorkManager("red")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        posi = manager.get_target_id()
        rospy.loginfo(posi)
        manager.pick()
        rate.sleep()
