#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manual.manual_command import ManualCommand
from catchrobo_manager.next_action_enum import (
    NextTarget,
    Action,
    PickAction,
    ShootAction,
)
from catchrobo_manager.work_manager import WorkManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.robot import Robot
from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.shooting_box_transform import ShootingBoxTransform

import rospkg
import rospy

from std_msgs.msg import Int8

import datetime
import pandas as pd
import numpy as np


class GameManager:
    def __init__(self):
        name_space = "game_manager/"

        self.MAX_HAS_WORK = rospy.get_param(name_space + "max_has_work")
        init_y_m_red = rospy.get_param(name_space + "init_y_m_red")
        self.INIT_Z_m = rospy.get_param(name_space + "INIT_Z_m")
        self.WORK_HEIGHT_m = rospy.get_param(name_space + "WORK_HEIGHT_m")
        shooting_box_center_red = rospy.get_param("calibration/shooting_box_center_red")

        self.SHOOT_HEIGHT_m = 0.01
        self.OPEN_A_BIT_RAD = np.deg2rad(10)
        self.FIELD = rospy.get_param("field")
        self.FIELD_SIGN = 1 if self.FIELD == "red" else -1
        self.INIT_Y_m = init_y_m_red * self.FIELD_SIGN
        shooting_box_center_raw = shooting_box_center_red * self.FIELD_SIGN

        self._shooting_box_transform = ShootingBoxTransform(shooting_box_center_raw)

        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber("menu", Int8, self.gui_callback)

    def init(self):
        # self._use_main_thread = False
        next_target = NextTarget.PICK
        self._work_manager = WorkManager(self.FIELD)
        self._box_manager = ShootingBoxManager(self.FIELD)
        self._robot = Robot(self.FIELD)

        self._target_work_info = self._work_manager.get_target_info()
        self._target_shoot_info = self._box_manager.get_target_info()

        posi, _, _ = self._target_work_info
        self.INIT_X_m = posi[0]
        self.BEFORE_COMMON_AREA_Y_m = posi[1]

        self._rate = rospy.Rate(10)
        self._gui_msg = GuiMenu.NONE
        self._manual_msg = ManualCommand.NONE

        self._go_flag = False
        self._old_my_area = True
        self._is_init_mode = True
        self._game_start = False
        self._log = []

    ########################################################################
    ### subscriber
    ########################################################################

    def gui_callback(self, msg):
        self._gui_msg = msg.data
        if self._gui_msg == GuiMenu.ORIGIN:
            self._robot.set_origin()
        elif self._gui_msg == GuiMenu.INIT:
            self.init_actions()
        elif self._gui_msg == GuiMenu.START:
            self.auto_mode()
            self._game_start = True
            self._game_start_t = rospy.Time.now()

        self._gui_msg = GuiMenu.NONE

    def manual_callback(self, msg):
        self._manual_msg = msg.data
        if self._manual_msg == ManualCommand.MANUAL_ON:
            self.manunal_mode()
        elif self._manual_msg == ManualCommand.MANUAL_OFF:
            self.auto_mode()
        # elif self._manual_msg == ManualCommand.GO:
        #     # self._go_flag = True
        #     self._robot.auto_mode()
        self._manual_msg = ManualCommand.NONE

    def auto_mode(self):
        self._robot.control_permission(True)
        rospy.loginfo("auto mode")

    def manunal_mode(self):
        self._robot.control_permission(False)
        rospy.loginfo("manual mode")

    ########################################################################
    ### control
    ########################################################################

    def temp(self, next_action: PickAction):
        work_position, is_my_area, target_id = self._work_manager.get_target_info()
        if target_id != self._last_target_id:
            if (
                next_action == PickAction.MOVE_Z_ON_WORK
                or next_action == PickAction.OPEN_GRIPPER
            ):
                ## MOVE_XY_ABOVE_WORK までは目標値が更新されるので問題なし
                next_action = PickAction.START
            elif (
                next_action == PickAction.MOVE_Z_TO_PICK
                or next_action == PickAction.PICK
            ):
                ## gripperを開けてから目標が消えたら
                self._robot.go(z=work_position[2])
                self._robot.close_gripper()
                self._robot.set_work_num(self._has_work)
                next_action = PickAction.START

        self._last_target_id = target_id

    def pick_actions(self, next_action: PickAction):
        pass_action = False  # 中断されようと、次に進むような動作
        next_target = NextTarget.PICK
        work_position, is_my_area, target_id = self._target_work_info

        if not self._work_manager.is_exist(target_id):
            ### 目標ワークがGUIで消された場合
            if (
                next_action == PickAction.MOVE_Z_TO_PICK
                or next_action == PickAction.PICK
            ):
                ### gripper open後なら、再度掴んでから
                self._robot.go(z=work_position[2])
                self._robot.close_gripper()
            ### リスタート
            next_action = PickAction.START

        if next_action == PickAction.START:
            self._target_work_info = self._work_manager.get_target_info()
        elif next_action == PickAction.MOVE_Z_SAFE:
            self._robot.go(z=self.INIT_Z_m)
        elif next_action == PickAction.STOP_BEFORE_COMMON:
            if is_my_area == False and self._old_my_area == True:
                ### 新たに共通エリアに入る場合
                self._robot.go(x=work_position[0], y=self.BEFORE_COMMON_AREA_Y_m)
                self.manunal_mode()
                pass_action = True
            self._old_my_area = is_my_area
        elif next_action == PickAction.MOVE_XY_ABOVE_WORK:
            self._robot.go(x=work_position[0], y=work_position[1])
        elif next_action == PickAction.MOVE_Z_ON_WORK:
            if self._robot.has_work():
                ### じゃがりこ重ねる
                self._robot.go(z=work_position[2] + self.WORK_HEIGHT_m)
        elif next_action == PickAction.OPEN_GRIPPER:
            # self._has_work = self._robot.has_work()
            # self._robot.set_work_num(0)
            self._robot.open_gripper()
        elif next_action == PickAction.MOVE_Z_TO_PICK:
            self._robot.go(z=work_position[2])
        elif next_action == PickAction.PICK:
            self._robot.pick()
            pass_action = True
            # self._robot.close_gripper()
            # self._robot.set_work_num(self._has_work + 1)
        elif next_action == PickAction.END:
            next_target = self._work_manager.pick(target_id)
            having_work = self._robot.has_work()
            if (
                having_work >= self.MAX_HAS_WORK
                or having_work >= self._box_manager.get_open_num()
            ):
                ### これ以上つかめなければshoot
                next_target = NextTarget.SHOOT
            next_action = 0  # 次はSTARTから始まる

        if self._robot.check_permission() or pass_action:
            ### action中にmanualに切り替わらず、動作を完遂したら、次の動作を行う
            next_action += 1

        return next_target, next_action

    def shoot_actions(self, next_action: ShootAction):
        pass_action = False
        next_target = NextTarget.SHOOT
        ### 目標シューティング位置計算
        box_position, target_id = self._target_shoot_info
        # box_position = self._shooting_box_transform.get_calibrated_position(
        #     box_position_raw
        # )
        if self._box_manager.is_exist(target_id):

            ### 目標がGUIで消された場合
            # if (
            #     next_action == ShootAction.SHOOT
            #     or next_action == ShootAction.PICK_WORK_ON_WORK
            # ):
            #     ### gripper open後なら、再度掴んでから
            ### リスタート
            next_action = PickAction.START

        if next_action == ShootAction.START:
            self._target_shoot_info = self._box_manager.get_target_info()
        elif next_action == ShootAction.MOVE_Z_SAFE:
            ### 上空へ上がる
            self._robot.go(z=self.INIT_Z_m)
        elif next_action == ShootAction.MOVE_XY_ABOVE_BOX:
            ### 穴上へxy移動
            self._robot.go(x=box_position[0], y=box_position[1], z=self.INIT_Z_m)
        # elif next_action == ShootAction.OPEN_A_BIT:
        #     self._robot.gripper(self.OPEN_A_BIT_RAD)
        elif next_action == ShootAction.MOVE_Z_TO_SHOOT:
            ### 下ろす
            self._robot.go(z=self.SHOOT_HEIGHT_m + box_position[2])
        elif next_action == ShootAction.PEG_IN_HOLE:
            ## グリグリ(手動)
            self.manunal_mode()
            pass_action = True
            ### ぐりぐり
            # self._robot.peg_in_hole()
            # shoot
        elif next_action == ShootAction.SHOOT:
            # self._has_work = self._robot.has_work() - 1
            # self._robot.open_gripper()
            # self._robot.set_work_num(0)
            self._robot.shoot()
            pass_action = True
        elif next_action == ShootAction.PICK_WORK_ON_WORK:
            if self._robot.has_work() > 0:
                ### 残ったじゃがりこを掴む
                self._robot.go(z=box_position[2] + self.WORK_HEIGHT_m)
                self._robot.close_gripper()
                # self._robot.set_work_num(self._has_work + 1)
        elif next_action == ShootAction.END:
            self._box_manager.shoot(target_id)
            if self._robot.has_work() == 0:
                ### 次のacution まだじゃがりこを持っていたらshoot, なければじゃがりこ掴み
                next_target = NextTarget.PICK
            next_action = 0  # 次はSTARTから始まる

        if self._robot.check_permission() or pass_action:
            ### action中にmanualに切り替わらず、動作を完遂したら、次の動作を行う
            next_action += 1

        return next_target, next_action

    def main_actions(self, next_target: NextTarget, next_action: Action):
        # rospy.loginfo("having work: {}".format(self._robot.has_work()))
        ### stop flagがたった瞬間に途中でも高速でループが終わる
        if next_target == NextTarget.PICK:
            if self._work_manager.get_remain_num() == 0:
                return NextTarget.SHOOT, ShootAction.START
            next_target, next_action = self.pick_actions(next_action)
        elif next_target == NextTarget.SHOOT:
            if self._box_manager.get_open_num() == 0:
                ### もうシュート場所がなければ終了
                return NextTarget.END, PickAction.START
            next_target, next_action = self.shoot_actions(next_action)

        return next_target, next_action

    def init_actions(self):
        rospy.loginfo("init action start")
        self._is_init_mode = True
        self.auto_mode()
        self._robot.enable()
        self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m, wait=False)
        self._robot.open_gripper()
        self._robot.close_gripper()
        self._robot.open_gripper()
        self._is_init_mode = False

        rospy.loginfo("init action finish")

    def end_actions(self):
        # 全部取り終わった
        rospy.loginfo("no open shooting box")
        self._robot.go(z=self.INIT_Z_m)
        # self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m)
        self.manunal_mode()

        rospy.loginfo("game_manager spin end")

    def spin(self):
        rospy.loginfo("game manager spin start")

        next_target = NextTarget.PICK
        next_action = PickAction.START
        while not rospy.is_shutdown():
            if (
                self._is_init_mode
                or not self._game_start
                or not self._robot.check_permission()
            ):
                ### init中 またはmanual モード中
                self._rate.sleep()
                old_t = rospy.Time.now()
                continue
            next_target, next_action = self.main_actions(next_target, next_action)
            t = rospy.Time.now()
            dt = t - old_t
            old_t = t
            log = [
                (t - self._game_start_t).to_sec(),
                dt.to_sec(),
                next_target,
                next_action,
            ]
            rospy.loginfo(log)
            self._log.append(log)
            if next_target == NextTarget.END:
                self.end_actions()
                break

        is_sim = rospy.get_param("sim")
        if is_sim:
            name = "sim/"
            if next_target != NextTarget.END:
                return
        else:
            name = "real/"
        self.savelog(name)

    def savelog(self, file_name: str = ""):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("catchrobo_manager")

        data_dir = pkg_path + "/log/" + file_name
        now = datetime.datetime.now()
        filename = data_dir + now.strftime("%Y%m%d_%H%M%S")
        df = pd.DataFrame(
            data=self._log,
            columns=[
                "time_s",
                "dt_s",
                "pick_or_shoot",
                "action_number",
            ],
        )
        df.to_csv(filename + ".csv", index=True)
        rospy.loginfo("save " + filename)


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # while not rospy.is_shutdown():
    ### 初期化
    game_manager.init()
    ### main動作
    game_manager.spin()
