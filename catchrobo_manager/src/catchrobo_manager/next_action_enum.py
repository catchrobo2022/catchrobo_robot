#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import IntEnum, auto


class NextTarget(IntEnum):
    PICK = auto()
    SHOOT = auto()
    SECOND_SHOOT = auto()
    WAIT_NEW = auto()
    END = auto()


class Action(IntEnum):
    pass


class PickAction(Action):
    START = 1
    MOVE_Z_SAFE = auto()  # 上空へ上がる
    STOP_BEFORE_COMMON = auto()  # 共通エリア前で一時停止
    MOVE_XY_ABOVE_WORK = auto()  # ワーク上空へ移動
    MOVE_Z_ON_WORK = auto()  # ワークを重ねる
    OPEN_GRIPPER = auto()  # グリッパー開く
    MOVE_Z_TO_PICK = auto()  # ワークをつかめる位置へ向かう
    PICK = auto()  # じゃがりこを掴む
    END = auto()  # 次は何をするかを計算


class ShootAction(Action):
    START = 1
    MOVE_Z_SAFE = auto()  # 上空へ上がる
    MOVE_XY_ABOVE_BOX = auto()  # 穴上へxy移動
    # OPEN_A_BIT = auto()
    MOVE_Z_TO_SHOOT = auto()  # 下ろす
    PEG_IN_HOLE = auto()  # ぐりぐり
    SHOOT = auto()
    PICK_WORK_ON_WORK = auto()
    END = auto()  # 次は何をするかを計算


class SecondShootAction(Action):
    START = 1
    MOVE_Z_SAFE = auto()  # 上空へ上がる
    MOVE_XY_ABOVE_BOX = auto()  # 穴上へxy移動
    MOVE_Z_TO_SHOOT = auto()  # 下ろす
    PEG_IN_HOLE = auto()
    SHOOT = auto()
    END = auto()
