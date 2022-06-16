#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tkinter.messagebox import NO
from catchrobo_driver.ros_cmd_template import RosCmdTemplate

import rospy
from std_msgs.msg import Int8
from catchrobo_msgs.msg import ErrorCode, EnableCmd, MyRosCmdArray, MyRosCmd
from sensor_msgs.msg import JointState


import math


class Motor:
    def __init__(self, id):
        self._running = False
        self._id = id
        self._ros_cmd_template = RosCmdTemplate()

        self._pub_ros_cmd = rospy.Publisher("ros_cmd", MyRosCmd, queue_size=1)

    def finish(self):
        self._running = False

    def go(self, target_position, has_work_num=0):
        self._running = True
        ros_command = self._ros_cmd_template.generate_ros_command(
            self._id, MyRosCmd.POSITION_CTRL_MODE, target_position, 0, has_work_num
        )
        self._pub_ros_cmd.publish(ros_command)

    def is_running(self):
        return self._running

    def peg_in_hole(self, has_work_num, z_threshold=None):
        self._running = True
        ros_command = self._ros_cmd_template.generate_ros_command(
            self._id, MyRosCmd.PEG_IN_HOLE_MODE, 0, 0, has_work_num
        )
        if z_threshold is not None:
            ros_command.position_min = z_threshold
        self._pub_ros_cmd.publish(ros_command)

    def set_origin(self, position, velocity, torque_threshold, has_work_num):
        self._running = True
        ros_command = self._ros_cmd_template.generate_ros_command(
            self._id, MyRosCmd.GO_ORIGIN_MODE, position, velocity, has_work_num
        )
        ros_command.acceleration_limit = torque_threshold
        self._pub_ros_cmd.publish(ros_command)


class Robot:
    def __init__(self):
        self.OPEN_GRIPPER_RAD = math.pi
        self.CLOSE_GRIPPER_RAD = 0
        self.PEG_IN_HOLE_THRESHOLD = 0.01
        self.TOUCH_POSITION = [0, 0, 0]  # [TODO]
        self.TOUCH_TORQUE_THRESHOLD = 0.3

        self._has_work = 0
        self._main_run_ok = False
        self._recovery_mode = False
        self._error = ErrorCode()
        self.N_MOTOR = 4
        self._rate = rospy.Rate(100)
        # self._joint_state = JointState()

        self._motors = [Motor(i) for i in range(self.N_MOTOR)]
        rospy.Subscriber("error", ErrorCode, self.error_callback)
        rospy.Subscriber("finihsed_flag_topic", Int8, self.finished_flag_callback)
        # rospy.Subscriber("my_joint_state", JointState, self.joint_state_callback)
        self._pub_enable = rospy.Publisher("enable_cmd", EnableCmd, queue_size=1)
        self._pub_joint_control = rospy.Publisher(
            "my_joint_control", MyRosCmdArray, queue_size=1
        )
        self._ros_cmd_template = RosCmdTemplate()
        self._enable_command = self._ros_cmd_template.generate_enable_command()

        # [TODO] 絶対座標→ロボット座標系変換
        # [TODO] 原点だしの位置

    # def joint_state_callback(self, msg):
    #     self._joint_state = msg

    def finished_flag_callback(self, msg):
        self._motors[msg.data].finish()

    ### 指示変更やめ
    def stop(self):
        self._main_run_ok = False

    def start(self):
        ### 動作開始
        ### recovery mode中ならしてから開始
        if not self._recovery_mode:
            self.enable(enable_check=True)
        self._main_run_ok = True

    ### 絶対座標系
    def go(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        if not self._main_run_ok:
            return
        if x is not None:
            self._motors[0].go(x, self._has_work)
        if y is not None:
            self._motors[1].go(y, self._has_work)
        if z is not None:
            self._motors[2].go(z, self._has_work)

        if wait:
            ### 全モーターの収束を待つ
            for i in range(len(self._motors)):
                self.wait_arrive(i)

    ### gripperを開く
    def open_gripper(self, wait=True):
        if not self._main_run_ok:
            return
        self._motors[3].go(self.OPEN_GRIPPER_RAD)

    def close_gripper(self, wait=True):
        if not self._main_run_ok:
            return
        self._motors[3].go(self.CLOSE_GRIPPER_RAD)

    def peg_in_hole(self):
        if not self._main_run_ok:
            return
        self._motors[0].peg_in_hole(self._has_work)
        self._motors[1].peg_in_hole(self._has_work)
        self._motors[2].peg_in_hole(self._has_work, self.PEG_IN_HOLE_THRESHOLD)

    def wait_arrive(self, id):
        while not rospy.is_shutdown():
            ### どれか1motorだけでも動き途中ならrunning > 0となる。
            ### ruuning == 0 まで待つ
            ### stop flagが立ったらwaitしない
            if self._motors[id].is_running() or not self._main_run_ok:
                break
            self._rate.sleep()

    def enable(self, enable_check=True):

        enable_command = self._ros_cmd_template.generate_enable_command()

        if self._error.error_code == ErrorCode.NONE:
            enable_command.enable_check = enable_check
        else:
            ### recovery mode. 復帰のためenable_checkは無し
            enable_command.enable_check = False

        self._pub_enable.publish(enable_command)

    def disable(self):
        self._main_run_ok = False
        enable_command = self._ros_cmd_template.generate_enable_command()
        enable_command.is_enable = False
        self._pub_enable.publish(enable_command)

    def has_work(self):
        return self._has_work

    def pick(self):
        self.close_gripper()
        self._has_work += 1

    def shoot(self):
        self.open_gripper()
        self._has_work -= 1

    def set_origin(self, id):
        self._main_run_ok = False
        self._motors[id].set_origin(
            self.TOUCH_POSITION[id], self.TOUCH_TORQUE_THRESHOLD
        )
        self.wait_arrive(id)

    def main_run_ok(self):
        return self._main_run_ok

    def error_callback(self, msg):
        self._main_run_ok = False
        self._error = msg
