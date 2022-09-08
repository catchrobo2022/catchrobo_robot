#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from catchrobo_manager.jagarico.database import Database
from catchrobo_manager.jagarico.gui_bridge import GuiBridge

import rospy


class ObjManagerBase(object):
    def __init__(self, calculator, csv_name):
        self._database = Database()
        self._calculator = calculator()
        self.EXIST_KEY = "exist"
        self._gui = None
        self.load(csv_name)

    def get_target_id(self):
        # self.update_by_gui()
        target_id = self._calculator.calcTarget(self._database)
        if self._gui is not None:
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
        return position, target_id, is_my_area

    def send_gui(self):
        if self._gui is not None:
            self._gui.sendGUI(self._database.getColumn(self.EXIST_KEY))

    def setCanGoCommon(self, flag):
        self._calculator.setCanGoCommon(flag)

    def update_by_gui(self, msg):
        for i, val in enumerate(msg.data):
            self._database.updateState(i, self.EXIST_KEY, bool(val))

    def get_remain_num(self, count_val):
        # self.update_by_gui()
        return self._database.count("exist", count_val)

    def is_exist(self, id: int):
        # self.update_by_gui()
        return self._database.isExist(id)

    def get_remain_num_in_common(self):
        return self._database.get_remain_num_in_common()

    def load(self, csv_name):
        self._database.readCsv(csv_name)

    def set_gui(self, gui_input_ros_output, ros_input_gui_output, target_topic):
        msg_template = [1] * self._database.getIdNum()
        self._gui = GuiBridge(
            gui_input_ros_output,
            ros_input_gui_output,
            target_topic,
            msg_template,
            self.update_by_gui,
        )
        rospy.sleep(0.3)  # publisher生成直後は送れないのでwait
        self.send_gui()

    def save_result(self, csv_name):
        self._database.save_csv(csv_name)
