#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import Field
from catchrobo_manual.manual_command import ManualCommand
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

from std_msgs.msg import Int8, Bool

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
        self.IS_SIM = rospy.get_param("sim")
        self.SHOOT_HEIGHT_m = rospy.get_param(name_space + "SHOOT_HEIGHT_m")
        self.IS_CONTINUE = rospy.get_param("is_continue")
        # game_end_topic = rospy.get_param("game_end_topic")

        # shooting_box_center_red = rospy.get_param("calibration/shooting_box_center_red")

        # self.OPEN_A_BIT_RAD = np.deg2rad(10)
        self.FIELD = rospy.get_param("field")
        self.FIELD_SIGN = 1 if self.FIELD == "red" else -1
        self.INIT_Y_m = init_y_m_red * self.FIELD_SIGN
        # shooting_box_center_raw = shooting_box_center_red * self.FIELD_SIGN

        # self._shooting_box_transform = ShootingBoxTransform(shooting_box_center_raw)
        # self._pub_game_end = rospy.Publisher(game_end_topic, Bool, queue_size=1)
        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber("menu", Int8, self.gui_callback, queue_size=1)

    def init(self):
        # self._use_main_thread = False
        top = "init/{}".format(self.FIELD)
        self._work_manager = WorkManager(top + "_work.csv")
        self._box_manager = ShootingBoxManager(top + "_shoot.csv")
        self._on_box_manager = ShootingBoxManager(top + "_on_shoot.csv")

        if self.IS_CONTINUE:
            top = "result/{}".format(self.FIELD)
            self._work_manager = WorkManager(top + "_work.csv")
            self._box_manager = ShootingBoxManager(top + "_shoot.csv")
            self._on_box_manager = ShootingBoxManager(top + "_on_shoot.csv")

        self._work_manager.set_gui("obj_giro", "obj_rigo", "target_work")
        self._box_manager.set_gui("gl_giro", "gl_rigo", "target_box")

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
        self._is_init_mode = False
        self._game_start = False
        self._log = []

    ########################################################################
    ### subscriber
    ########################################################################

    def gui_callback(self, msg):
        if msg.data == GuiMenu.ORIGIN:
            self._robot.set_origin()
            self._robot.open_gripper()
        elif msg.data == GuiMenu.INIT:
            self._is_init_mode = True
            self._game_start = False
            # self.init_actions()
        elif msg.data == GuiMenu.START:
            self.auto_mode()
            self._game_start = True
            self._game_start_t = rospy.Time.now()
        elif msg.data == GuiMenu.CALIBRATION:
            self._robot.close_gripper()
        elif msg.data == GuiMenu.POINT4:
            rospy.sleep(1)
            self._robot.open_gripper()

    def manual_callback(self, msg):
        # self._manual_msg = msg.data
        if msg.data == ManualCommand.MANUAL_ON:
            self.manunal_mode()
        elif msg.data == ManualCommand.MANUAL_OFF:
            self.auto_mode()
        elif msg.data == ManualCommand.START:
            self.auto_mode()
            self._game_start = True
            self._game_start_t = rospy.Time.now()
        # elif self._manual_msg == ManualCommand.GO:
        #     # self._go_flag = True
        #     self._robot.auto_mode()
        # self._manual_msg = ManualCommand.NONE

    def auto_mode(self):
        self._robot.control_permission(True)
        rospy.loginfo("auto mode")

    def manunal_mode(self):
        self._robot.control_permission(False)
        rospy.loginfo("manual mode")

    ########################################################################
    ### control
    ########################################################################

    def pick_actions(self, next_action: PickAction):
        pass_action = False  # 中断されようと、次に進むような動作
        next_target = NextTarget.PICK
        work_position, target_id, is_my_area = self._target_work_info

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
                self._robot.ask_manual()
                pass_action = True
            self._old_my_area = is_my_area
        elif next_action == PickAction.MOVE_XY_ABOVE_WORK:
            self._robot.go(x=work_position[0], y=work_position[1])
        elif next_action == PickAction.MOVE_Z_ON_WORK:
            if self._robot.has_work():
                ### じゃがりこ重ねる
                self._robot.go(z=work_position[2] + self.WORK_HEIGHT_m)
                pass
        elif next_action == PickAction.OPEN_GRIPPER:
            self._robot.open_gripper()
            # self._robot.open_gripper(wait=False)

        elif next_action == PickAction.MOVE_Z_TO_PICK:
            self._robot.go(z=work_position[2])
        elif next_action == PickAction.PICK:
            self._robot.pick()
            pass_action = True
            # self._robot.close_gripper()
            # self._robot.set_work_num(self._has_work + 1)
        elif next_action == PickAction.END:
            self._work_manager.pick(target_id)
            next_target = self.calc_next_target_after_pick(target_id)
            next_action = 0  # 次はSTARTから始まる
            self._old_my_area = is_my_area

        if self._robot.check_permission() or pass_action:
            ### action中にmanualに切り替わらず、動作を完遂したら、次の動作を行う
            next_action += 1

        return next_target, next_action

    def calc_next_target_after_pick(self, pick_id):
        if (
            (pick_id == 0)
            or (pick_id == 19)
            or (pick_id == 18)
            or (pick_id == 13)
            or (pick_id == 24)
        ):
            next_target = NextTarget.SHOOT
        else:
            next_target = NextTarget.PICK

        having_work = self._robot.has_work()
        if (
            having_work >= self.MAX_HAS_WORK
            or having_work >= self._box_manager.get_remain_num()
        ):
            ### これ以上つかめなければshoot
            next_target = NextTarget.SHOOT
        return next_target

    def shoot_actions(self, next_action: ShootAction):
        pass_action = False
        next_target = NextTarget.SHOOT
        ### 目標シューティング位置計算
        box_position, target_id, is_my_area = self._target_shoot_info
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
            self._robot.go(z=self.SHOOT_HEIGHT_m)
        elif next_action == ShootAction.PEG_IN_HOLE:
            ## グリグリ(手動)
            self._robot.ask_manual()
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
                self._robot.go(z=self.PICK_WORK_ON_SHOOTING_BOX_m)
                self._robot.close_gripper()
                # self._robot.set_work_num(self._has_work + 1)
        elif next_action == ShootAction.END:
            self._box_manager.shoot(target_id)
            if self._robot.has_work() == 0:
                ### 次のacution まだじゃがりこを持っていたらshoot, なければじゃがりこ掴み
                next_target = NextTarget.PICK
            next_action = 0  # 次はSTARTから始まる
            self._old_my_area = True

        if self._robot.check_permission() or pass_action:
            ### action中にmanualに切り替わらず、動作を完遂したら、次の動作を行う
            next_action += 1

        return next_target, next_action

    def second_shoot_actions(self, next_action: Action):
        pass_action = False
        next_target = NextTarget.SECOND_SHOOT
        ### 目標シューティング位置計算
        box_position, target_id, _ = self._target_second_shoot_info
        # box_position = self._shooting_box_transform.get_calibrated_position(
        #     box_position_raw
        # )
        if self._on_box_manager.is_exist(target_id):

            ### 目標がGUIで消された場合
            # if (
            #     next_action == ShootAction.SHOOT
            #     or next_action == ShootAction.PICK_WORK_ON_WORK
            # ):
            #     ### gripper open後なら、再度掴んでから
            ### リスタート
            next_action = SecondShootAction.START

        if next_action == SecondShootAction.START:
            self._target_second_shoot_info = self._on_box_manager.get_target_info()
        elif next_action == SecondShootAction.MOVE_Z_SAFE:
            ### 上空へ上がる
            self._robot.go(z=self.INIT_Z_m)
        elif next_action == SecondShootAction.MOVE_XY_ABOVE_BOX:
            ### 穴上へxy移動
            self._robot.go(x=box_position[0], y=box_position[1], z=self.INIT_Z_m)
        # elif next_action == ShootAction.OPEN_A_BIT:
        #     self._robot.gripper(self.OPEN_A_BIT_RAD)
        elif next_action == SecondShootAction.MOVE_Z_TO_SHOOT:
            ### 下ろす
            self._robot.go(z=self.SECOND_SHOOT_m)
        elif next_action == SecondShootAction.PEG_IN_HOLE:
            ## グリグリ(手動)
            self._robot.ask_manual()
            pass_action = True
            ### ぐりぐり
            # self._robot.peg_in_hole()
            # shoot
        elif next_action == SecondShootAction.SHOOT:
            # self._has_work = self._robot.has_work() - 1
            # self._robot.open_gripper()
            # self._robot.set_work_num(0)
            self._robot.shoot()
            pass_action = True
        elif next_action == SecondShootAction.END:
            self._on_box_manager.shoot(target_id)
            next_action = 0  # 次はSTARTから始まる
            next_target = NextTarget.PICK
            self._old_my_area = True

        if self._robot.check_permission() or pass_action:
            ### action中にmanualに切り替わらず、動作を完遂したら、次の動作を行う
            next_action += 1

        return next_target, next_action

    def main_actions(self, next_target: NextTarget, next_action: Action):
        ### stop flagがたった瞬間に途中でも高速でループが終わる
        if next_target == NextTarget.PICK:
            if self._work_manager.get_remain_num() == 0:
                return NextTarget.SHOOT, ShootAction.START
            next_target, next_action = self.pick_actions(next_action)
        elif next_target == NextTarget.SHOOT:
            if self._box_manager.get_remain_num() == 0:
                ### もうシュート場所がなければ終了
                return NextTarget.SECOND_SHOOT, PickAction.START
            next_target, next_action = self.shoot_actions(next_action)
        elif next_target == NextTarget.SECOND_SHOOT:
            if self._on_box_manager.get_remain_num() == 0:
                return NextTarget.END, PickAction.START
            next_target, next_action = self.second_shoot_actions(next_action)

        return next_target, next_action

    def init_actions(self):
        rospy.loginfo("init action start")
        self._is_init_mode = True
        self._game_start = False

        self.auto_mode()
        self._robot.enable()
        self._robot.set_accel_scale("init")
        self._robot.go(z=self.INIT_Z_m)
        self._robot.go(x=self.INIT_X_m)
        self._robot.go(y=self.INIT_Y_m)
        self._robot.open_gripper()
        self._robot.close_gripper()
        self._robot.open_gripper()
        self._robot.set_accel_scale("normal")
        self._is_init_mode = False
        if not self.IS_CONTINUE:
            top = "calibrated/{}_".format(self.FIELD)
            self._box_manager.load(top + "shoot.csv")
            self._on_box_manager.load(top + "on_shoot.csv")

        rospy.loginfo("init action finish")

    def end_actions(self):
        # 全部取り終わった
        rospy.loginfo("no open shooting box")
        self._robot.go(z=self.INIT_Z_m)
        # self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m)
        self._robot.ask_manual()

        rospy.loginfo("game_manager spin end")

    def change_current_scale(self):
        if self._work_manager.get_remain_num_in_common() >= 5:
            self._robot.set_accel_scale("fast")
        else:
            self._robot.set_accel_scale("normal")

    def spin(self):
        rospy.loginfo("game manager spin start")

        next_target = NextTarget.PICK
        next_action = PickAction.START
        while not rospy.is_shutdown():
            if self._is_init_mode:
                self.init_actions()
            if (
                self._is_init_mode
                or not self._game_start
                or not self._robot.check_permission()
            ):
                ### init中 またはmanual モード中
                self._rate.sleep()
                old_t = rospy.Time.now()
                continue
            self.change_current_scale()
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
        self.save(next_target)

    def save(self, next_target):
        self.save_obj()
        is_sim = self.IS_SIM
        if is_sim:
            name = "sim/"
            if next_target != NextTarget.END:
                return
        else:
            name = "real/"
        self.savelog(name)

    def save_obj(self):
        self._work_manager.save_result("result/" + self.FIELD + "_work.csv")
        self._box_manager.save_result("result/" + self.FIELD + "_shoot.csv")
        self._box_manager.save_result("result/" + self.FIELD + "_on_shoot.csv")

    def savelog(self, file_name=""):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("catchrobo_log")

        data_dir = pkg_path + "/log/action_time/" + file_name
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

        ### ca


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # while not rospy.is_shutdown():
    ### 初期化
    game_manager.init()
    ### main動作
    game_manager.spin()
