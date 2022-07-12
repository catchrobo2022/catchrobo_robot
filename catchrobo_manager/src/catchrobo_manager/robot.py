#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.ros_cmd_template import RosCmdTemplate
from catchrobo_manager.robot_transform import WorldRobotTransform
from catchrobo_manager.motor import Motor

import rospy
from std_msgs.msg import Int8, Bool
from catchrobo_msgs.msg import (
    ErrorCode,
    EnableCmd,
    MyRosCmdArray,
    MyRosCmd,
    PegInHoleCmd,
)
from sensor_msgs.msg import JointState


import math


### input : ワールド座標系
class Robot:
    def __init__(self, field):
        self.FIELD = field

        self.OPEN_GRIPPER_RAD = math.pi
        self.CLOSE_GRIPPER_RAD = 0
        self.PEG_IN_HOLE_THRESHOLD_ROBOT = 0.01

        self._has_work = 0
        self._main_run_ok = False
        self._recovery_mode = False
        self._error = ErrorCode()
        self.N_MOTOR = 4
        self._rate = rospy.Rate(100)
        self._current_state_robot = JointState()

        self._motors = [Motor(i) for i in range(self.N_MOTOR)]
        rospy.Subscriber("error", ErrorCode, self.error_callback)
        rospy.Subscriber("finished_flag_topic", Int8, self.finished_flag_callback)
        rospy.Subscriber("my_joint_state", JointState, self.joint_state_callback)
        self._pub_enable = rospy.Publisher("enable_cmd", EnableCmd, queue_size=1)
        self._pub_joint_control = rospy.Publisher(
            "my_joint_control", MyRosCmdArray, queue_size=1
        )
        self._pub_peg_in_hole_cmd = rospy.Publisher(
            "peg_in_hole_cmd", Bool, queue_size=1
        )

        self._ros_cmd_template = RosCmdTemplate()
        self._enable_command = self._ros_cmd_template.generate_enable_command()
        #  絶対座標→ロボット座標系変換
        self._world_robot_transform = WorldRobotTransform(self.FIELD)

    def world2robot_transform(self, id, position):
        return position

    def joint_state_callback(self, msg):
        self._current_state_robot = msg
        self._current_state_robot.position = list(self._current_state_robot.position)
        self._current_state_robot.position[1] *= 2  # y軸は2倍動く

    def finished_flag_callback(self, msg):
        print(msg)
        self._motors[msg.data].finish()

    ### 指示変更やめ
    def mannual_on(self):
        self._main_run_ok = False
        for i in range(len(self._motors)):
            self._motors[i].finish()

        ### peg in hole mode中の可能性があるので強制終了する
        peg_cmd = self._ros_cmd_template.generate_peg_in_hole_command([0, 0, 0])
        peg_cmd.data = False
        self._pub_peg_in_hole_cmd.publish(peg_cmd)

    def start(self):
        ### 動作開始
        ### recovery mode中ならenableしてから開始
        if self._recovery_mode:
            self.enable(enable_check=True)
        self._main_run_ok = True

    ## robot座標系
    def go_robot_m(self, robot_position, wait=True):
        for i, val in enumerate(robot_position):
            self._motors[i].go(val, self._has_work)

        if wait:
            ### 全モーターの収束を待つ
            for i in range(len(self._motors)):
                self.wait_arrive(i)

    def run_motor(self, id, robot_position_m):
        self._motors[id].go(robot_position_m, self._has_work)

    def wait_all(self):
        for i in range(len(self._motors)):
            self.wait_arrive(i)

    ### 絶対座標系
    def go(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        if not self._main_run_ok:
            return

        positions = [x, y, z]
        for id, position in enumerate(positions):
            if position is not None:
                robot_m = self._world_robot_transform.world2robot_each(id, position)
                self.run_motor(id, robot_m)
        if wait:
            self.wait_all()

    ### gripperを開く
    def open_gripper(self, wait=True):
        if not self._main_run_ok:
            return
        self._motors[3].go(self.OPEN_GRIPPER_RAD)
        if wait:
            self.wait_arrive(3)

    def close_gripper(self, wait=True):
        if not self._main_run_ok:
            return
        self._motors[3].go(self.CLOSE_GRIPPER_RAD)
        if wait:
            self.wait_arrive(3)

    def peg_in_hole(self):
        if not self._main_run_ok:
            return
        current_state_robot = self._current_state_robot.position
        peg_cmd = self._ros_cmd_template.generate_peg_in_hole_command(
            current_state_robot
        )
        rospy.loginfo(peg_cmd)
        self._pub_peg_in_hole_cmd.publish(peg_cmd)
        for i in range(3):
            self._motors[i].direct_control(current_state_robot[i], 0, self._has_work)

        for i in range(len(self._motors)):
            self.wait_arrive(i)

    def wait_arrive(self, id):
        while not rospy.is_shutdown() and self._motors[id].is_running():
            self._rate.sleep()

    def enable(self, enable_check=True):

        enable_command = self._ros_cmd_template.generate_enable_command(
            True, enable_check
        )
        self._pub_enable.publish(enable_command)
        print(enable_command)
        rospy.sleep(1)

    def disable(self):
        self.mannual_on()
        enable_command = self._ros_cmd_template.generate_enable_command(False)
        self._pub_enable.publish(enable_command)

    def has_work(self):
        return self._has_work

    def pick(self):
        self.close_gripper()
        self._has_work += 1

    def shoot(self):
        self.open_gripper()
        self._has_work -= 1

    def set_origin(self):
        rospy.loginfo("set origin")
        self.set_origin_each(0)
        rospy.sleep(0.3)
        self.set_origin_each(1)
        rospy.sleep(0.3)
        self.set_origin_each(2)
        rospy.sleep(0.3)

    def set_origin_each(self, id):
        self._motors[id].set_origin(0)

    def main_run_ok(self):
        return self._main_run_ok

    def error_callback(self, msg):
        if msg.error_code == ErrorCode.FINISH:
            id = msg.id
            self._motors[id].finish()
        else:
            self.mannual_on()
            self._error = msg
