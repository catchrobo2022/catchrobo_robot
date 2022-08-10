#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.ros_cmd_template import RosCmdTemplate
from catchrobo_manager.robot_transform import WorldRobotTransform
from catchrobo_manager.motor import Motor
from catchrobo_manual.manual_command import ManualCommand

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

        ##### [TODO] ros param化
        name_space = "robot/"
        robot_origin_m = rospy.get_param(name_space + "robot_origin_m")
        self.OPEN_GRIPPER_RAD = rospy.get_param(name_space + "OPEN_GRIPPER_RAD")
        self.CLOSE_GRIPPER_RAD = rospy.get_param(name_space + "CLOSE_GRIPPER_RAD")
        finish_wait_Hz = rospy.get_param(name_space + "finish_wait_Hz")
        self.IS_SIM = rospy.get_param("sim")

        ############################

        # self.SERVO_WAIT_s = 0.5

        self._has_work = 0
        self._control_permission = False
        # self._recovery_mode = False
        # self._error = ErrorCode()
        ### [WARN] 4想定でプログラム作成しており、可変にはできていません
        self.N_MOTOR = 4
        self._rate = rospy.Rate(finish_wait_Hz)

        # self._current_state_robot = JointState()
        self._ros_cmd_template = RosCmdTemplate()
        if self.IS_SIM:
            self._ros_cmd_template.set_current_state([1, 1, 1, 1])

        self._motors = [Motor(i, self._ros_cmd_template) for i in range(self.N_MOTOR)]
        rospy.Subscriber("error", ErrorCode, self.error_callback)
        # rospy.Subscriber("finished_flag_topic", Int8, self.finished_flag_callback)
        # rospy.Subscriber("my_joint_state", JointState, self.joint_state_callback)
        self._pub_enable = rospy.Publisher("enable_cmd", EnableCmd, queue_size=1)
        # self._pub_joint_control = rospy.Publisher(
        #     "my_joint_control", MyRosCmdArray, queue_size=1
        # )
        # self._pub_peg_in_hole_cmd = rospy.Publisher(
        #     "peg_in_hole_cmd", Bool, queue_size=1
        # )

        self._enable_command = self._ros_cmd_template.generate_enable_command()
        #  絶対座標→ロボット座標系変換
        self._world_robot_transform = WorldRobotTransform(self.FIELD, robot_origin_m)

        manual_command_topic = "auto2manual_command"
        self._pub_manual_command = rospy.Publisher(
            manual_command_topic, Int8, queue_size=1
        )

    # def world2robot_transform(self, id, position):
    #     return position

    # def joint_state_callback(self, msg):
    #     self._current_state_robot = msg
    #     self._current_state_robot.position = list(self._current_state_robot.position)
    #     self._current_state_robot.position[1] *= 2  # y軸は2倍動く

    # def finished_flag_callback(self, msg):
    #     print(msg)
    #     self._motors[msg.data].finish()

    # def mannual_mode(self):
    #     ### 使わない
    #     self._control_permission = False
    #     for i in range(len(self._motors)):
    #         self._motors[i].finish()

    #     ### peg in hole mode中の可能性があるので強制終了する
    #     peg_cmd = self._ros_cmd_template.generate_peg_in_hole_command([0, 0, 0])
    #     peg_cmd.data = False
    #     self._pub_peg_in_hole_cmd.publish(peg_cmd)

    # def auto_mode(self):
    #     ### 使わない
    #     ### 動作開始
    #     ### recovery mode中ならenableしてから開始
    #     if self._recovery_mode:
    #         self.enable(enable_check=True)
    #     self._control_permission = True

    def control_permission(self, ok: bool):
        self._control_permission = ok
        # if ok is False:
        #     ### motorを止める
        #     for i in range(len(self._motors)):
        #         self._motors[i].finish()

        # ### peg in hole mode中の可能性があるので強制終了する
        # peg_cmd = self._ros_cmd_template.generate_peg_in_hole_command([0, 0, 0])
        # peg_cmd.data = False
        # self._pub_peg_in_hole_cmd.publish(peg_cmd)

    def set_current_limit_scale(self, scale):
        self._ros_cmd_template.set_current_state(scale)

    ## robot座標系
    def go_robot_m(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        if not self._control_permission:
            return

        positions = [x, y, z]
        for id, position in enumerate(positions):
            if position is not None:
                # robot_m = self._world_robot_transform.world2robot_each(id, position)
                self.run_motor(id, position)
        if wait:
            self.wait_all()

    def run_motor(self, id, robot_position_m):
        self._motors[id].go(robot_position_m, self._has_work)

    def wait_all(self):
        for i in range(3):
            self.wait_arrive(i)

    ### 絶対座標系
    def go(self, x=None, y=None, z=None, wait=True):
        ### stop flagが立ったら指示しない
        if not self._control_permission:
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
        # if not self._control_permission:
        #     return
        self._motors[3].go(self.OPEN_GRIPPER_RAD)
        if wait:
            self.wait_arrive(3)
            # rospy.sleep(self.SERVO_WAIT_s)

    def close_gripper(self, wait=True):
        # if not self._control_permission:
        #     return
        self._motors[3].go(self.CLOSE_GRIPPER_RAD)
        if wait:
            # rospy.sleep(self.SERVO_WAIT_s)
            self.wait_arrive(3)

    def peg_in_hole(self):
        if not self._control_permission:
            return
        current_state_robot = self._current_state_robot.position
        peg_cmd = self._ros_cmd_template.generate_peg_in_hole_command(
            current_state_robot
        )
        self._pub_peg_in_hole_cmd.publish(peg_cmd)
        for i in range(3):
            self._motors[i].direct_control(current_state_robot[i], 0, self._has_work)

        for i in range(len(self._motors)):
            self.wait_arrive(i)

    def wait_arrive(self, id):
        while (
            not rospy.is_shutdown()
            and self._motors[id].is_running()
            and self._control_permission
        ):
            self._rate.sleep()

    def enable(self, enable_check=True):

        enable_command = self._ros_cmd_template.generate_enable_command(
            True, enable_check
        )
        self._pub_enable.publish(enable_command)
        print(enable_command)
        rospy.sleep(1)

    def disable(self):
        # self.mannual_mode()
        enable_command = self._ros_cmd_template.generate_enable_command(False)
        self._pub_enable.publish(enable_command)
        self.ask_manual()

    def has_work(self):
        return self._has_work

    def add_work(self, val: int):
        self._has_work += val

    def set_work_num(self, num: int):
        self._has_work = num

    def pick(self):
        self.close_gripper()
        self.add_work(1)

    def shoot(self):
        self.open_gripper()
        self.add_work(-1)

    def set_origin(self):
        rospy.loginfo("set origin start")
        self.set_origin_each(0)
        rospy.sleep(0.3)
        self.set_origin_each(1)
        rospy.sleep(0.3)
        self.set_origin_each(2)
        rospy.sleep(0.3)
        rospy.loginfo("set origin finish")

    def set_origin_each(self, id):
        self._motors[id].set_origin(self.FIELD)

    # def main_run_ok(self):
    #     ### 使わない
    #     return self._control_permission

    def error_callback(self, msg):
        if msg.error_code == ErrorCode.FINISH:
            id = msg.id
            self._motors[id].finish()
        else:
            # self.mannual_mode()
            self.ask_manual()
            # self._error = msg

    def ask_manual(self):
        if self.IS_SIM:
            return
        self.control_permission(False)
        msg = Int8()
        msg.data = ManualCommand.MANUAL_ON
        self._pub_manual_command.publish(msg)

    def check_permission(self):
        return self._control_permission

    def gripper(self, rad: float, wait=True):
        self._motors[3].go(rad)
        if wait:
            # rospy.sleep(self.SERVO_WAIT_s)
            self.wait_arrive(3)
