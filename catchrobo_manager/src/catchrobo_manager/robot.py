#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_msgs.msg import ErrorCode, EnableCmd, MyRosCmdArray, MyRosCmd
from std_msgs.msg import Int8


class Motor:
    def __init__(self):
        self._running = False

    def finish(self):
        self._running = False


class Robot:
    def __init__(self):
        self._has_work = 0
        self._main_run_ok = False
        self._error = None

        self._motors = [Motor() for i in range(4)]
        rospy.Subscriber("error", ErrorCode, self.error_callback)
        rospy.Subscriber("finihsed_flag_topic", Int8, self.finished_flag_callback)
        self._pub_enable = rospy.Publisher("enable_cmd", EnableCmd, queue_size=1)
        self._pub_joint_control = rospy.Publisher(
            "my_joint_control", MyRosCmdArray, queue_size=1
        )

    # [TODO]
    def error_callback(self, msg):
        self._main_run_ok = False
        self._error = msg

    # [TODO]
    def finished_flag_callback(self, msg):
        self._motors[msg.data].finish()

    ### 指示変更やめ
    def stop(self):
        self._main_run_ok = False

    ### 動作開始
    def start(self):
        self._main_run_ok = True

    # [TODO]
    ### 絶対座標系
    def go(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        if not self._main_run_ok:
            return

    # [TODO]
    ### gripperを開く
    def open_gripper(self, wait=True):
        if not self._main_run_ok:
            return

    def close_gripper(self, wait=True):
        if not self._main_run_ok:
            return

    # [TODO]
    def peg_in_hole(self):
        if not self._main_run_ok:
            return

    # [TODO]
    def wait_arrive(self, id):
        ### stop flagが立ったらwaitしない
        if not self._main_run_ok:
            return

    # [TODO]
    def enable(self):
        if self._error is None:
            pass
        else:
            self.recovery(self._error.id)

    def recovery(self, id):
        pass

    # [TODO]
    def disable(self):
        self._main_run_ok = False

    def has_work(self):
        return self._has_work

    # [TODO]
    def pick(self):
        self.close_gripper()
        self._has_work += 1

    # [TODO]
    def shoot(self):
        self.open_gripper()
        self._has_work -= 1

    # [TODO]
    def set_origin(self, id):
        self._main_run_ok = False

    def main_run_ok(self):
        return self._main_run_ok
