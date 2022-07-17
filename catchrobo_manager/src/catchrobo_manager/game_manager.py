#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manual.manual_command import ManualCommand
from catchrobo_manager.next_action_enum import NextAction
from catchrobo_manager.work_manager import WorkManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.robot import Robot


from std_msgs.msg import Int8


class GuiMenu:
    NONE = 0
    ORIGIN = 1
    CALIBLATION = 2
    POINT1 = 3
    POINT2 = 4
    POINT3 = 5
    INIT = 6
    START = 7


class GameManager:
    def __init__(self):
        # [TODO] ros paramで受け取る

        self.Z_ABOVE_ZYAGARIKO = 0.14
        self.DIFF_Z_ZYAGARIKO_PUT = 0.3
        self.MAX_HAS_WORK = 5
        self.FIELD = "red"
        self.FIELD_SIGN = 1 if self.FIELD == "red" else -1
        self.INIT_Y_m = 1.7 * self.FIELD_SIGN
        self.INIT_Z_m = 0.205
        self.SAFE_Z_m = self.INIT_Z_m
        self.WORK_HEIGHT = 0.087
        self.BEFORE_COMMON_AREA_Y_m = 0.3

        # self._use_main_thread = False
        next_target = NextAction.PICK
        self._work_manager = WorkManager(self.FIELD)
        self._box_manager = ShootingBoxManager(self.FIELD)
        self._robot = Robot(self.FIELD)

        posi, _ = self._work_manager.get_target_info()

        self.INIT_X_m = posi[0]

        self._rate = rospy.Rate(10)
        self._gui_msg = GuiMenu.NONE
        self._manual_msg = ManualCommand.NONE

        self._go_flag = False
        self._old_my_area = True
        self._init_mode = True
        rospy.Subscriber("manual_command", Int8, self.manual_callback)
        rospy.Subscriber("menu", Int8, self.gui_callback)

    def gui_callback(self, msg):
        self._gui_msg = msg.data
        if self._gui_msg == GuiMenu.ORIGIN:
            self._robot.set_origin()
        elif self._gui_msg == GuiMenu.INIT:
            self.init_actions()
        elif self._gui_msg == GuiMenu.START:
            self._robot.start()

        self._gui_msg = GuiMenu.NONE

    def manual_callback(self, msg):
        self._manual_msg = msg.data
        if self._manual_msg == ManualCommand.MANUAL_ON:
            self._robot.mannual_on()
        elif self._manual_msg == ManualCommand.MANUAL_OFF:
            self._robot.start()
        # elif self._manual_msg == ManualCommand.GO:
        #     # self._go_flag = True
        #     self._robot.start()
        self._manual_msg = ManualCommand.NONE

    def main_actions(self, next_target):

        ### stop flagがたった瞬間に途中でも高速でループが終わる
        ### じゃがりこ掴む
        if next_target == NextAction.PICK:
            rospy.loginfo("Go to work")
            #### [WARN] zは安全域スタートの想定
            ### 目標じゃがりこ計算
            work_position, is_my_area = self._work_manager.get_target_info()
            print(is_my_area)
            if is_my_area is False and self._old_my_area is True:
                ### 新たに共通エリアに入る場合
                rospy.loginfo("go to common area")
                self._robot.go(
                    x=work_position[0], y=self.BEFORE_COMMON_AREA_Y_m, z=self.SAFE_Z_m
                )
                self._old_my_area = is_my_area
                # next_target = NextAction.WAIT_GO_SIGN
                self._robot.mannual_on()
                return NextAction.PICK
            self._old_my_area = is_my_area
            ### じゃがりこへxy移動
            self._robot.go(x=work_position[0], y=work_position[1], z=self.SAFE_Z_m)
            if self._robot.has_work():
                ### じゃがりこ重ねる
                self._robot.go(z=work_position[2] + self.WORK_HEIGHT)
            ### グリッパー開く
            self._robot.open_gripper()
            ### じゃがりこをつかめる位置へ行く
            self._robot.go(z=work_position[2])
            ### つかむ
            rospy.loginfo("pick")
            self._robot.pick()
            next_target = self._work_manager.pick()
            ### 上空へ上がる
            self._robot.go(z=self.SAFE_Z_m)
            having_work = self._robot.has_work()
            if (
                having_work > self.MAX_HAS_WORK
                or having_work >= self._box_manager.get_open_num()
            ):
                ### これ以上つかめなければshoot
                next_target = NextAction.SHOOT

        # シュート
        elif next_target == NextAction.SHOOT:
            if self._box_manager.get_open_num() == 0:
                ### もうシュート場所がなければ終了
                return NextAction.END
            rospy.loginfo("Go to shooting box")
            ### 目標シューティング位置計算
            box_position = self._box_manager.get_target_info()
            ### 穴上へxy移動
            self._robot.go(x=box_position[0], y=box_position[1], z=self.SAFE_Z_m)
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
                self._robot.go(z=box_position[2] + self.WORK_HEIGHT)
                self._robot.close_gripper()

            ### 上空へ上がる
            self._robot.go(z=self.SAFE_Z_m)
            ### グリグリ(手動)
            # self._robot.mannual_on()

        # 全部取り終わった
        elif next_target == NextAction.END:
            self._robot.go(z=self.SAFE_Z_m)
            # self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m)
            self._robot.mannual_on()

        return next_target

    def init_actions(self):
        rospy.loginfo("init action start")
        self._init_mode = True
        # [TODO] 厳密な値
        self._robot.start()
        self._robot.enable()
        self._robot.go(self.INIT_X_m, self.INIT_Y_m, self.INIT_Z_m, wait=False)
        self._robot.open_gripper()
        self._robot.close_gripper()
        self._robot.open_gripper()
        self._robot.mannual_on()
        self._init_mode = False

        rospy.loginfo("init action finish")

    def spin(self):
        # ### callback呼び出しのため、別スレッドで重複呼び出しの可能性がある。
        # if self._use_main_thread:
        #     ### 重複していたら新規は作らない
        #     return
        # self._use_main_thread = True
        rospy.loginfo("game manager spin start")
        next_target = NextAction.PICK
        while not rospy.is_shutdown():
            if self._robot.main_run_ok() and self._init_mode is False:
                next_target = self.main_actions(next_target)
            else:
                self._rate.sleep()


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    game_manager.spin()
