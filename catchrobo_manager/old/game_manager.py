#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manual.manual_command import ManualCommand
from catchrobo_manager.next_action_enum import NextAction
from catchrobo_manager.work_manager import WorkManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.robot import Robot
from catchrobo_manager.gui_menu_enum import GuiMenu
from catchrobo_manager.shooting_box_transform import ShootingBoxTransform


from std_msgs.msg import Int8


class GameManager:
    def __init__(self):
        name_space = "game_manager/"

        self.MAX_HAS_WORK = rospy.get_param(name_space + "max_has_work")
        init_y_m_red = rospy.get_param(name_space + "init_y_m_red")
        self.INIT_Z_m = rospy.get_param(name_space + "INIT_Z_m")
        self.WORK_HEIGHT_m = rospy.get_param(name_space + "WORK_HEIGHT_m")
        shooting_box_center_red = rospy.get_param("calibration/shooting_box_center_red")

        self.FIELD = rospy.get_param("field")
        self.FIELD_SIGN = 1 if self.FIELD == "red" else -1
        self.INIT_Y_m = init_y_m_red * self.FIELD_SIGN
        shooting_box_center_raw = shooting_box_center_red * self.FIELD_SIGN

        self._shooting_box_transform = ShootingBoxTransform(shooting_box_center_raw)

        ######### [TODO] ros paramで受け取る
        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber("menu", Int8, self.gui_callback)

    def init(self):
        # self._use_main_thread = False
        next_target = NextAction.PICK
        self._work_manager = WorkManager(self.FIELD)
        self._box_manager = ShootingBoxManager(self.FIELD)
        self._robot = Robot(self.FIELD)

        posi, _, _ = self._work_manager.get_target_info()

        self.INIT_X_m = posi[0]
        self.BEFORE_COMMON_AREA_Y_m = posi[1]

        self._rate = rospy.Rate(10)
        self._gui_msg = GuiMenu.NONE
        self._manual_msg = ManualCommand.NONE

        self._go_flag = False
        self._old_my_area = True
        self._is_init_mode = True
        self._game_start = False

    def gui_callback(self, msg):
        self._gui_msg = msg.data
        if self._gui_msg == GuiMenu.ORIGIN:
            self._robot.set_origin()
        elif self._gui_msg == GuiMenu.INIT:
            self.init_actions()
        elif self._gui_msg == GuiMenu.START:
            self.auto_mode()
            self._game_start = True

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

    def main_actions(self, next_target):
        rospy.loginfo("having work: {}".format(self._robot.has_work()))
        ### stop flagがたった瞬間に途中でも高速でループが終わる
        if next_target == NextAction.PICK:
            ### じゃがりこ掴む
            rospy.loginfo("Go to work")
            #### [WARN] zは安全域スタートの想定
            ### 目標じゃがりこ計算
            work_position, is_my_area, target_id = self._work_manager.get_target_info()
            rospy.loginfo("target work : {}".format(target_id))
            # print(is_my_area)
            ## pandasを使っているためか、is 演算子が使えない。==を使う
            # if is_my_area == False and self._old_my_area == True:
            #     # if is_my_area is False and self._old_my_area is True:
            #     self._old_my_area = is_my_area
            #     ### 新たに共通エリアに入る場合
            #     rospy.loginfo("go to common area")
            #     self._robot.go(
            #         x=work_position[0], y=self.BEFORE_COMMON_AREA_Y_m, z=self.INIT_Z_m
            #     )
            #     # next_target = NextAction.WAIT_GO_SIGN
            #     self._robot.mannual_mode()
            #     return NextAction.PICK
            self._old_my_area = is_my_area
            ### じゃがりこへxy移動
            self._robot.go(x=work_position[0], y=work_position[1], z=self.INIT_Z_m)
            if self._robot.has_work():
                ### じゃがりこ重ねる
                self._robot.go(z=work_position[2] + self.WORK_HEIGHT_m)
            ### グリッパー開く
            self._robot.open_gripper()
            ### じゃがりこをつかめる位置へ行く
            self._robot.go(z=work_position[2])
            ### つかむ
            rospy.loginfo("pick")
            self._robot.pick()
            next_target = self._work_manager.pick(target_id)
            ### 上空へ上がる
            self._robot.go(z=self.INIT_Z_m)
            having_work = self._robot.has_work()
            if (
                having_work > self.MAX_HAS_WORK
                or having_work >= self._box_manager.get_open_num()
            ):
                ### これ以上つかめなければshoot
                next_target = NextAction.SHOOT
            rospy.loginfo("pick finish")
        # シュート
        elif next_target == NextAction.SHOOT:
            rospy.loginfo("Go to shooting box")
            if self._box_manager.get_open_num() == 0:
                ### もうシュート場所がなければ終了
                return NextAction.END
            ### 目標シューティング位置計算
            box_position = self._box_manager.get_target_info()
            # box_position = self._shooting_box_transform.get_calibrated_position(
            #     box_position_raw
            # )
            ### 穴上へxy移動
            self._robot.go(x=box_position[0], y=box_position[1], z=self.INIT_Z_m)
            ### 下ろす
            self._robot.go(z=box_position[2])
            ### ぐりぐり
            # self._robot.peg_in_hole()
            # shoot
            rospy.loginfo("shoot")
            self._robot.shoot()
            self._box_manager.shoot()

            if self._robot.has_work() == 0:
                ### 次のacution まだじゃがりこを持っていたらshoot, なければじゃがりこ掴み
                next_target = NextAction.PICK
            else:
                ### 残ったじゃがりこを掴む
                self._robot.go(z=box_position[2] + self.WORK_HEIGHT_m)
                self._robot.close_gripper()

            ### 上空へ上がる
            self._robot.go(z=self.INIT_Z_m)
            ### グリグリ(手動)
            # self._robot.mannual_mode()

        # 全部取り終わった
        elif next_target == NextAction.END:
            rospy.loginfo("no open shooting box")
            self._robot.go(z=self.INIT_Z_m)
            # self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m)
            self._robot.mannual_mode()

        return next_target

    def init_actions(self):
        rospy.loginfo("init action start")
        self._is_init_mode = True
        # [TODO] 厳密な値
        # self._robot.control_permission(True)
        self.auto_mode()
        self._robot.enable()
        self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m, wait=False)
        self._robot.open_gripper()
        self._robot.close_gripper()
        self._robot.open_gripper()
        # self._robot.control_permission(False)
        self._is_init_mode = False

        rospy.loginfo("init action finish")

    def auto_mode(self):
        self._robot.control_permission(True)
        rospy.loginfo("auto mode")

    def manunal_mode(self):
        self._robot.control_permission(False)
        rospy.loginfo("manual mode")

    def spin(self):
        # ### callback呼び出しのため、別スレッドで重複呼び出しの可能性がある。
        # if self._use_main_thread:
        #     ### 重複していたら新規は作らない
        #     return
        # self._use_main_thread = True
        rospy.loginfo("game manager spin start")

        next_target = NextAction.PICK
        while not rospy.is_shutdown():
            if (
                self._is_init_mode
                or not self._game_start
                or not self._robot.check_permission()
            ):
                ### init中 またはmanual モード中
                self._rate.sleep()
                continue

            next_target = self.main_actions(next_target)
            if next_target == NextAction.END:
                break

        rospy.loginfo("game_manager spin end")


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    # while not rospy.is_shutdown():
    ### 初期化
    game_manager.init()
    ### main動作
    game_manager.spin()
