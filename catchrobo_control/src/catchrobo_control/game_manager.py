#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_control.mannual_command import ManualCommand
from catchrobo_control.next_action_enum import NextAction
from catchrobo_control.work_manager import WorkManager
from catchrobo_control.shooting_box_manager import ShootingBoxManager

from std_msgs.msg import Int8


class ActionResult:
    DOING = 0
    FINISH = 1
    GAME_END = 2
    PERMISSION = 3
    SHOOT_PERMISSION = 4


class Robot:
    def __init__(self):
        self._has_work = 0
        pass

    # [TODO]
    ### 指示変更やめ
    def stop(self):
        pass

    # [TODO]
    ### 動作開始
    def start(self):
        pass

    # [TODO]
    ### 絶対座標系
    def go(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        pass

    # [TODO]
    ### gripperを開く
    def open_gripper(self, wait=True):
        pass

    def close_gripper(self, wait=True):
        pass

    # [TODO]
    def wait_arrive(self, id):
        ### stop flagが立ったらwaitしない
        pass

    # [TODO]
    def enable(self):
        pass

    # [TODO]
    def disable(self):
        pass

    def has_work(self):
        return self._has_work

    # [TODO]
    def pick(self):
        self.close_gripper()
        self._has_work += 1

    # [TODO]
    def peg_in_hole(self):
        pass

    # [TODO]
    def shoot(self):
        self.open_gripper()
        self._has_work -= 1

    # [TODO]
    def set_origin(self, id):
        pass


class GameManager:
    def __init__(self):
        # [TODO] ros paramで受け取る
        self.Z_ABOVE_ZYAGARIKO = 1
        self.DIFF_Z_ZYAGARIKO_PUT = 0.3
        self.MAX_HAS_WORK = 5
        self._field = "red"

        self._use_main_thread = False
        self._next_target = NextAction.PICK
        self._work_manager = WorkManager(self._field)
        self._box_manager = ShootingBoxManager(self._field)
        self._robot = Robot()

        rospy.Subscriber("manual", Int8, self.manualCallback)

    def manualCallback(self, msg):
        if msg.data == ManualCommand.START:
            self.main()
        elif msg.data == ManualCommand.ENABLE:
            self.enable()
        elif msg.data == ManualCommand.DISABLE:
            self.disable()
        elif msg.data == ManualCommand.STOP:
            self.stop()
        elif msg.data == ManualCommand.SET_ORIGIN:
            self.set_origin()

    def stop(self):
        self._stop_flag = True
        self._robot.stop()

    def set_origin(self):
        ### enable
        self._robot.enable()
        ### z軸setorigin
        self._robot.set_origin(2)
        ### y軸 setorigin
        self._robot.set_origin(1)
        ### x軸 setorign
        self._robot.set_origin(0)
        ### グリッパー開く
        self._robot.open_gripper()
        ### じゃがりこにあたらない高さまでzを下ろす
        self._robot.go(z=self.Z_ABOVE_ZYAGARIKO)

    def enable(self):
        self._robot.enable()

    def disable(self):
        self._robot.disable()

    def main(self):
        ### callback呼び出しのため、別スレッドで重複呼び出しの可能性がある。
        if self._use_main_thread:
            ### 重複していたら新規は作らない
            return
        self._use_main_thread = True

        self._stop_flag = False
        self._robot.start()
        self._robot.enable()
        while not rospy.is_shutdown() and not self._stop_flag:
            ### stop flagがたった瞬間に途中でも高速でループが終わるようにする

            ### じゃがりこ掴む
            if self._next_target == NextAction.PICK:
                ### 目標じゃがりこ計算
                work_position = self._work_manager.get_target()
                ### 上空へ上がる
                self._robot.go(z=self.Z_ABOVE_ZYAGARIKO)
                ### じゃがりこへxy移動
                self._robot.go(x=work_position[0], y=work_position[1])
                if self._robot.has_work():
                    ### じゃがりこ重ねる
                    self._robot.go(z=self.DIFF_Z_ZYAGARIKO_PUT + work_position[2])
                ### グリッパー開く
                self._robot.open_gripper()
                ### じゃがりこをつかめる位置へ行く
                self._robot.go(z=work_position[2])
                ### つかむ
                self._robot.pick()
                ### 次のacution じゃがりこを一列取りきったらshoot,
                self._next_target = self._work_manager.pick()
                ### これ以上つかめなければshoot
                if self._robot.has_work() > self.MAX_HAS_WORK:
                    self._next_target = NextAction.SHOOT

            # シュート
            elif self._next_target == NextAction.SHOOT:
                ### 目標シューティング位置計算
                box_position = self._box_manager.get_target()
                ### 上空へ上がる
                self._robot.go(z=self.Z_ABOVE_ZYAGARIKO)
                ### 穴上へxy移動
                self._robot.go(x=box_position[0], y=box_position[1])
                ### 下ろす
                self._robot.go(z=work_position[2])
                ### ぐりぐり
                self._robot.peg_in_hole()
                # shoot
                self._robot.shoot()
                self._box_manager.shoot()
                ### 次のacution まだじゃがりこを持っていたらshoot, なければじゃがりこ掴み
                if self._robot.has_work() == 0:
                    self._next_target = NextAction.PICK
                ### もうシュート場所がなければ終了
                if self._box_manager.get_open_num() == 0:
                    self._next_target = NextAction.END

            # 全部取り終わった
            elif self._next_target == NextAction.END:
                self._robot.go(z=self.Z_ABOVE_ZYAGARIKO)
                self._robot.go(0, 0, 0)
                pass


if __name__ == "__main__":
    rospy.init_node("GameManager")
    game_manager = GameManager()
    game_manager.init()
    rospy.spin()
