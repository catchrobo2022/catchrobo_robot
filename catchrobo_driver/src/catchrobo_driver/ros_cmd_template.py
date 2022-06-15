#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.src.catchrobo_driver.rad_transform import RadTransform
from catchrobo_msgs.msg import EnableCmd, MyRosCmd

import rospkg

import pandas as pd


class RosCmdTemplate:
    def readCsv(self, color):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_driver")
        config_path = pkg_path + "/config/"
        csv = config_path + "default_limit.csv"
        self._datas = pd.read_csv(csv, index_col=0)

    def __init__(self):
        self._rad_transform = RadTransform()

        rad_transform = self._rad_transform
        command = MyRosCmd()

        command.id = id  # 0: x軸 1:y軸 2: z軸 3:グリッパー
        command.mode = mode  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.DIRECT_CTRL_MODE
        command.position = rad_transform.robot_m2rad(command.id, robot_position)
        command.velocity = rad_transform.robot_m2rad(
            command.id, robot_velocity
        )  # 目標位置での速度

        command.net_inertia = 0.0  # 慣性モーメント(未対応)
        command.effort = 0  # 自重補償項(未対応)
        command.position_min = rad_transform.robot_m2rad(command.id, 0.0)  # 可動域
        command.position_max = rad_transform.robot_m2rad(command.id, 1.5)  # 可動域
        command.velocity_limit = rad_transform.robot_m2rad(
            command.id, 1.0
        )  # 台形加速中の最大速度
        command.acceleration_limit = rad_transform.robot_m2rad(
            command.id, 2.0
        )  # 台形加速を作るための最大加速度
        command.jerk_limit = rad_transform.robot_m2rad(
            command.id, 5.0
        )  # 台形加速を作るための最大躍度(加速度の微分)
        command.kp = 5  # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
        command.kd = 0.5

    def enable(self):
        rad_transform = self._rad_transform

        ### 本当に危険なとき用の制約を設定
        enable_command = EnableCmd()
        ### motor電源を入れる
        enable_command.is_enable = True
        ### x, y, zの順
        ### 可動域
        enable_command.position_min = [0.0, rad_transform.robot_m2rad(1, -0.44125), 0]
        enable_command.position_max = [
            rad_transform.robot_m2rad(0, 1.3525),
            rad_transform.robot_m2rad(1, 0.44125),
            rad_transform.robot_m2rad(2, 0.145),
        ]
        ### 速度制約
        enable_command.velocity_limit = [
            rad_transform.robot_m2rad(i, 3.093972094) for i in range(3)
        ]
        ### トルク制約
        enable_command.torque_limit = [
            rad_transform.robot_m2rad(i, 32.57947937) for i in range(3)
        ]
        ### 目標位置とどれくらいかけ離れていたらだめか
        enable_command.trajectory_error_limit = [
            rad_transform.robot_m2rad(i, 0.5) for i in range(3)
        ]

        ### 障害物としての箱を設定。 x,y,zの値
        enable_command.obstacle_min = [1] * 3  # min > maxにすると、制約無しとなる. 面倒なので今はなし
        enable_command.obstacle_max = [-1] * 3

        return enable_command

    def generate(self, id, mode, robot_position, robot_velocity):
        rad_transform = RadTransform()
        command = MyRosCmd()

        command.id = id  # 0: x軸 1:y軸 2: z軸 3:グリッパー
        command.mode = mode  # MyRosCmd.POSITION_CTRL_MODE or MyRosCmd.DIRECT_CTRL_MODE
        command.position = rad_transform.robot_m2rad(command.id, robot_position)
        command.velocity = rad_transform.robot_m2rad(
            command.id, robot_velocity
        )  # 目標位置での速度

        command.net_inertia = 0.0  # 慣性モーメント(未対応)
        command.effort = 0  # 自重補償項(未対応)
        command.position_min = rad_transform.robot_m2rad(command.id, 0.0)  # 可動域
        command.position_max = rad_transform.robot_m2rad(command.id, 1.5)  # 可動域
        command.velocity_limit = rad_transform.robot_m2rad(
            command.id, 1.0
        )  # 台形加速中の最大速度
        command.acceleration_limit = rad_transform.robot_m2rad(
            command.id, 2.0
        )  # 台形加速を作るための最大加速度
        command.jerk_limit = rad_transform.robot_m2rad(
            command.id, 5.0
        )  # 台形加速を作るための最大躍度(加速度の微分)
        command.kp = 5  # p += cmd.kp * (cmd.p - p) + cmd.kd * cmd.v * dt
        command.kd = 0.5

        return command
