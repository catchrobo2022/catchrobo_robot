#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_control.next_action_enum import NextAction
from catchrobo_control.zyagariko_manager import ZyagarikoManager


class ActionResult():
    DOING = 0
    FINISH = 1
    GAME_END = 2
    PERMISSION = 3
    SHOOT_PERMISSION = 4


class CatchroboCenter():
    def __init__(self):
        self._use_main_thread = False
        self._next_target = NextAction.PICK
        self._zyagariko_manager = ZyagarikoManager(self._field)


    #[TODO]
    def setupMove(self):
        ### enable
        ### setorigin
        ### z軸を上げておく
        pass

    def start(self):
        self._stop_flag = False
        self._robot.stop(False)

    def stop(self):
        self._stop_flag = True
        self._robot.stop(True)


    def main(self):
        ### callback呼び出しのため、別スレッドで重複呼び出しの可能性がある。
        if(self._use_main_thread):
            ### 重複していたら新規は作らない
            return
        self._use_main_thread = True 

        self.start()
        while not rospy.is_shutdown() and not self._stop_flag:
            
            #[TODO]
            ### じゃがりこ掴む
            if(self._next_target == NextAction.PICK):
                ### 目標じゃがりこ計算
                position, next_action = self._zyagariko_manager.pick()
                ### 上空へ上がる


                ### じゃがりこ上へ進む
                ### 下ろす
                ### つかむ

                ### 次のacution じゃがりこを一列取りきったらshoot, 
                pass

            # シュート
            elif(self._next_target == NextAction.SHOOT):
                pass
                ### 目標シューティング位置計算
                ### 上空へ上がる
                ### 穴上へ進む
                ### 下ろす
                ### ぐりぐり

                ### 次のacution まだじゃがりこを持っていたらshoot, なければじゃがりこ掴み

            ### stop時 はここで終わる


