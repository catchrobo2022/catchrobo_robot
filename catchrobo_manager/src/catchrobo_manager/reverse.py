#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.game_manager import GameManager
from catchrobo_manager.next_action_enum import (
    NextTarget,
    Action,
    PickAction,
    ShootAction,
    SecondShootAction,
)
from catchrobo_manager.work_manager import WorkManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.on_box_manager import OnBoxManager
from catchrobo_manager.robot import Robot
from catchrobo_manager.gui_menu_enum import GuiMenu

import rospkg
import rospy

from std_msgs.msg import Int8

import datetime
import pandas as pd
import numpy as np


class ReverseManager(GameManager):
    def init(self):
        # self._use_main_thread = False
        top = "reverse/{}".format(self.FIELD)
        self._work_manager = WorkManager(top + "_shoot.csv")
        self._box_manager = ShootingBoxManager(top + "_work.csv")
        self._box_manager.set_gui("obj_giro", "obj_rigo", "target_work")
        self._work_manager.set_gui("gl_giro", "gl_rigo", "target_box")
        self._on_box_manager = ShootingBoxManager(top + "_on_shoot.csv")

        self._robot = Robot(self.FIELD)

        self._target_work_info = self._work_manager.get_target_info()
        self._target_shoot_info = self._box_manager.get_target_info()
        self._target_second_shoot_info = self._on_box_manager.get_target_info()

        posi, _, _ = self._target_work_info
        self.INIT_X_m = posi[0]
        self.BEFORE_COMMON_AREA_Y_m = posi[1]
        self.PICK_WORK_ON_SHOOTING_BOX_m = self.WORK_HEIGHT_m + posi[2]
        self.SECOND_SHOOT_m = self.PICK_WORK_ON_SHOOTING_BOX_m

        self._rate = rospy.Rate(10)
        # self._manual_msg = ManualCommand.NONE

        self._go_flag = False
        self._old_my_area = True
        self._is_init_mode = True
        self._game_start = False
        self._log = []
        self.IS_SIM = True
        self.IS_CONTINUE = True

    ########################################################################
    ### control
    ########################################################################

    def main_actions(self, next_target: NextTarget, next_action: Action):
        # rospy.loginfo("having work: {}".format(self._robot.has_work()))
        ### stop flagがたった瞬間に途中でも高速でループが終わる
        if next_target == NextTarget.PICK:
            if self._work_manager.get_remain_num() == 0:
                return NextTarget.SHOOT, ShootAction.START
            if self._box_manager.get_remain_num() == 0:
                return NextTarget.END, PickAction.START
            next_target, next_action = self.pick_actions(next_action)
        elif next_target == NextTarget.SHOOT:
            if self._work_manager.get_remain_num() == 0 and self._robot.has_work() == 0:
                ### もう掴んでいなければ終了
                return NextTarget.END, PickAction.START
            if self._box_manager.get_remain_num() == 0:
                return NextTarget.END, PickAction.START
            next_target, next_action = self.shoot_actions(next_action)

        return next_target, next_action

    def init_actions(self):
        rospy.loginfo("init action start")
        self._is_init_mode = True
        self.auto_mode()
        self._robot.enable()
        self._robot.set_accel_scale("init")
        self._robot.go(z=self.INIT_Z_m)
        self._robot.go(x=self.INIT_X_m)
        # self._robot.go(y=self.INIT_Y_m)
        self._robot.open_gripper()
        self._robot.close_gripper()
        self._robot.open_gripper()
        self._robot.set_accel_scale("normal")
        self._is_init_mode = False

        rospy.loginfo("init action finish")

    def change_current_scale(self):
        self._robot.set_accel_scale("normal")

    def save(self, next_target):
        return

    def calc_next_target_after_pick(self, pick_id):
        next_target = NextTarget.PICK
        having_work = self._robot.has_work()
        if (
            having_work >= self.MAX_HAS_WORK
            or having_work >= self._box_manager.get_remain_num()
        ):
            ### これ以上つかめなければshoot
            next_target = NextTarget.SHOOT
        return next_target

    def end_actions(self):
        # 全部取り終わった
        rospy.loginfo("no open shooting box")
        self._robot.go(z=self.INIT_Z_m)
        self._robot.go(y=self.INIT_Y_m * 0.6)
        # self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m)
        self._robot.ask_manual()

        rospy.loginfo("game_manager spin end")


if __name__ == "__main__":
    rospy.init_node("ReverseManager")
    game_manager = ReverseManager()
    # while not rospy.is_shutdown():
    ### 初期化
    game_manager.init()
    ### main動作
    game_manager.spin()
