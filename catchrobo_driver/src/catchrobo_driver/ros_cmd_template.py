#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.rad_transform import RadTransform
from catchrobo_msgs.msg import EnableCmd, MyRosCmd
from scipy import constants


import rospkg

import pandas as pd


class RosCmdTemplate:
    def __init__(self):
        self._work_mass = 0.06
        self._rad_transform = RadTransform()
        self._datas = self.readCsv()
        # print(self._datas)
        # print(self._datas.loc["position_min"][0])

    def readCsv(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_driver")
        config_path = pkg_path + "/config/"
        csv = config_path + "default_limit.csv"
        datas = pd.read_csv(csv, index_col=0)
        return datas

    def generate_enable_command(self):
        rad_transform = self._rad_transform

        ### 本当に危険なとき用の制約を設定
        enable_command = EnableCmd()
        ### motor電源を入れる
        enable_command.is_enable = True
        ### x, y, zの順
        ### 可動域
        enable_command.position_min = [
            rad_transform.robot_m2rad(i, val)
            for i, val in enumerate(self._datas.loc["enable_position_min"])
        ]

        enable_command.position_max = [
            rad_transform.robot_m2rad(i, val)
            for i, val in enumerate(self._datas.loc["enable_position_max"])
        ]
        ### 速度制約
        enable_command.velocity_limit = [
            rad_transform.robot_m2rad(i, val)
            for i, val in enumerate(self._datas.loc["enable_velocity_limit"])
        ]
        ### トルク制約
        enable_command.torque_limit = [
            rad_transform.robot_m2rad(i, val)
            for i, val in enumerate(self._datas.loc["enable_torque_limit"])
        ]
        ### 目標位置とどれくらいかけ離れていたらだめか
        enable_command.trajectory_error_limit = [
            rad_transform.robot_m2rad(i, 0.5) for i in range(3)
        ]

        ### 障害物としての箱を設定。 x,y,zの値
        enable_command.obstacle_min = [1] * 3  # min > maxにすると、制約無しとなる. 面倒なので今はなし
        enable_command.obstacle_max = [-1] * 3

        return enable_command

    def generate_ros_command(
        self, id, mode, robot_position, robot_end_velocity=0, has_work_num=0
    ):
        rad_transform = self._rad_transform
        command = MyRosCmd()
        command.id = id
        command.position_min = rad_transform.robot_m2rad(
            id, self._datas.loc["position_min"][id]
        )
        command.position_max = rad_transform.robot_m2rad(
            id, self._datas.loc["position_max"][id]
        )
        command.velocity_limit = rad_transform.robot_m2rad(
            id, self._datas.loc["velocity_limit"][id]
        )
        command.acceleration_limit = rad_transform.robot_m2rad(
            id, self._datas.loc["acceleration_limit"][id]
        )
        command.jerk_limit = rad_transform.robot_m2rad(
            id, self._datas.loc["jerk_limit"][id]
        )
        command.kp = self._datas.loc["kp"][id]
        command.kd = self._datas.loc["kd"][id]

        command.mode = mode
        command.position = rad_transform.robot_m2rad(command.id, robot_position)
        command.velocity = rad_transform.robot_m2rad(
            command.id, robot_end_velocity
        )  # 目標位置での速度

        mass = sum(self._datas.loc["mass"][id:]) + self._work_mass * has_work_num
        r = rad_transform.get_pulley_radius(id)
        inertia = self._datas.loc["inertia"][id]
        command.net_inertia = inertia + mass * r

        if id == 2:
            command.effort = mass * constants.G * r
        return command

    def robot_m2rad(self, motor_id, position):
        return self._rad_transform.robot_m2rad(motor_id, position)
