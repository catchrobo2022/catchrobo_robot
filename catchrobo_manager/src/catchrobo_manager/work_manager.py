#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from catchrobo_manager.next_action_enum import NextTarget
from catchrobo_manager.jagarico.database import Database
from catchrobo_manager.jagarico.target_jagarico_calculator import (
    TargetJagaricoCalculator,
)
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
        self._gui = GuiBridge(
            "obj_giro", "obj_rigo", "target_work", msg_template, self.update_by_gui
        )
        self._gui.sendGUI(self._database.getColumn(self.EXIST_KEY))

    def get_target_id(self):
        # self.update_by_gui()
        target_id = self._calculator.calcTarget(self._database)
        self._gui.send_target(target_id)
        return target_id

    def get_target_info(self):

        target_id = self.get_target_id()
        # 全部のじゃがりこを取り終わるとNoneをcalcTarget()が返すので場合分け
        if target_id == None:
            position = None
            is_my_area = None
        else:
            position = self._database.getPosi(target_id)
            is_my_area = self._database.getState(target_id, "my_area")

        # print(target_id)
        # print(is_my_area)
        return position, is_my_area, target_id

    def pick(self, pick_id):
        # pick_id = self.get_target_id()
        # self.update_by_gui()
        self._database.delete(pick_id)
        self._gui.sendGUI(self._database.getColumn(self.EXIST_KEY))

        ## 次動作計算アルゴリズム
        ## もうシュートするなら
        # next_action = NextTarget.SHOOT
        ## 次も連続してじゃがりこを回収するなら
        # next_action = NextTarget.PICK
        if (
            (pick_id == 0)
            or (pick_id == 19)
            or (pick_id == 18)
            or (pick_id == 13)
            or (pick_id == 24)
        ):
            next_action = NextTarget.SHOOT
        else:
            next_action = NextTarget.PICK

        return next_action

    def setCanGoCommon(self, flag):
        self._calculator.setCanGoCommon(flag)

    def update_by_gui(self, msg):
        # msg = self._gui.getMsg()
        for i, val in enumerate(msg.data):
            # rospy.loginfo("i, val {}{}".format(i,val))
            self._database.updateState(i, self.EXIST_KEY, bool(val))

    def get_remain_num(self):
        # self.update_by_gui()
        return self._database.count("exist", True)

    def is_exist(self, id: int):
        # self.update_by_gui()
        return self._database.isExist(id)

    def get_remain_num_in_common(self):
        return self._database.get_remain_num_in_common()


if __name__ == "__main__":
    import rospy

    rospy.init_node("test")
    manager = WorkManager("red")
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():

        posi = manager.get_target_id()
        rospy.loginfo(posi)
        manager.pick()
        rate.sleep()
