#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.obj_manager_base import ObjManagerBase
from catchrobo_manager.jagarico.target_shooting_box_calculator import (
    TargetShootingBoxCalculator,
)
import rospy


class ShootingBoxManager(ObjManagerBase):
    def __init__(self, csv_name):
        super().__init__(TargetShootingBoxCalculator, csv_name)

    def shoot(self, id):
        self._database.updateState(id, "exist", True)
        self.send_gui()
        if self.get_remain_num() > 0:
            self.get_target_id()

    def get_remain_num(self):
        # self.update_by_gui()
        return super().get_remain_num(False)


if __name__ == "__main__":
    import rospy

    rospy.init_node("test")
    manager = ShootingBoxManager("red")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        id = manager.get_target_id()
        rospy.loginfo(id)
        manager.shoot()
        rate.sleep()
