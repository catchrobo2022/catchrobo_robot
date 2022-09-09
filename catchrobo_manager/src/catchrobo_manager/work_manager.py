#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.obj_manager_base import ObjManagerBase
from catchrobo_manager.jagarico.target_jagarico_calculator import (
    TargetJagaricoCalculator,
)
import rospy


class WorkManager(ObjManagerBase):
    def __init__(self, csv_name):
        super().__init__(TargetJagaricoCalculator, csv_name)

    def pick(self, pick_id):
        self._database.delete(pick_id)
        self.send_gui()
        if self.get_remain_num() > 0:
            self.get_target_id()

    def get_remain_num(self):
        # self.update_by_gui()
        return super().get_remain_num(True)


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
