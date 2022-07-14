#!/usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import Bool
from catchrobo_driver.rad_transform import RadTransform
from catchrobo_msgs.msg import EnableCmd, MyRosCmd, PegInHoleCmd
from scipy import constants


import rospkg

import pandas as pd
import math


class RosCmdTemplate:
    def __init__(self):
        self._work_mass = 0.06
        self._velocity_limit_scale = 1
        self._accerelation_limit_scale = 1
        self._jerk_limit = 100

        ## キレイに動いたときのパラメーター 07/14
        # self._velocity_limit_scale = 1
        # self._accerelation_limit_scale = 0.4
        # self._jerk_limit = 20

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

    def generate_enable_command(self, is_enable=False, enable_check=True):
        rad_transform = self._rad_transform

        ### 本当に危険なとき用の制約を設定
        enable_command = EnableCmd()
        ### motor電源を入れる
        enable_command.is_enable = is_enable
        ### 可動域外だとdisableになる安全機能のonoff
        enable_command.enable_check = enable_check
        ### x, y, zの順
        ### 可動域
        # enable_command.position_min = [
        #     rad_transform.robot_m2rad(i, val)
        #     for i, val in enumerate(self._datas.loc["enable_position_min"])
        # ]

        # enable_command.position_max = [
        #     rad_transform.robot_m2rad(i, val)
        #     for i, val in enumerate(self._datas.loc["enable_position_max"])
        # ]
        # ### 速度制約
        # enable_command.velocity_limit = [
        #     rad_transform.robot_m2rad(i, val)
        #     for i, val in enumerate(self._datas.loc["enable_velocity_limit"])
        # ]
        # ### トルク制約
        # enable_command.torque_limit = [
        #     rad_transform.robot_m2rad(i, val)
        #     for i, val in enumerate(self._datas.loc["enable_torque_limit"])
        # ]
        # ### 目標位置とどれくらいかけ離れていたらだめか
        # enable_command.trajectory_error_limit = [
        #     rad_transform.robot_m2rad(i, 0.5) for i in range(3)
        # ]

        # ### 障害物としての箱を設定。 x,y,zの値
        # enable_command.obstacle_min = [1] * 3  # min > maxにすると、制約無しとなる. 面倒なので今はなし
        # enable_command.obstacle_max = [-1] * 3

        return enable_command

    def generate_ros_command(
        self, id, mode, robot_position, robot_end_velocity=0, has_work_num=0
    ):
        rad_transform = self._rad_transform
        command = MyRosCmd()
        command.id = id
        command.position_min = self._datas.loc["position_min_rad"][id]
        command.position_max = self._datas.loc["position_max_rad"][id]
        command.velocity_limit = (
            self._datas.loc["velocity_limit_rad"][id] * self._velocity_limit_scale
        )
        command.acceleration_limit = (
            self._datas.loc["acceleration_limit_rad"][id]
            * self._accerelation_limit_scale
        )
        command.jerk_limit = self._jerk_limit
        command.kp = self._datas.loc["kp"][id]
        command.kd = self._datas.loc["kd"][id]

        command.mode = mode
        command.position = rad_transform.robot_m2rad(command.id, robot_position)
        # 目標位置での終端速度
        command.velocity = rad_transform.robot_m2rad(command.id, robot_end_velocity)

        mass = sum(self._datas.loc["mass"][id:]) + self._work_mass * has_work_num
        r = rad_transform.get_pulley_radius(id)
        inertia = self._datas.loc["inertia"][id]
        command.net_inertia = inertia + mass * r

        if id == 2:
            command.effort = mass * constants.G * r

        return command

    def robot_m2rad(self, motor_id, position):
        return self._rad_transform.robot_m2rad(motor_id, position)

    def generate_peg_in_hole_command(self, current_position_robot):
        # command = PegInHoleCmd()
        # command.run = True
        # # [TODO] ros paramにする
        # command.radius_delta = self.robot_m2rad(0, 0.009)
        # command.target_velocity = 2 * math.pi * command.radius_delta * 2
        # t = 5
        # command.max_radius = math.sqrt(t * command.target_velocity / (2 * math.pi))
        # command.z_threshold = 0.05
        # command.center_x = self.robot_m2rad(0, current_position_robot[0])
        # command.center_y = self.robot_m2rad(1, current_position_robot[1])
        command = Bool()
        command.data = True
        return command

    def generate_origin_command(self, id, velocity_m):
        command = self.generate_ros_command(
            id, MyRosCmd.GO_ORIGIN_MODE, 0, velocity_m, 0
        )
        command.position = self._datas.loc["origin_position_rad"][id]
        command.acceleration_limit = self._datas.loc["origin_torque_threshold_rad"][id]
        return command

    def generate_velocity_command(self, id, velocity_m, has_work_num=0):
        command = self.generate_ros_command(
            id, MyRosCmd.DIRECT_CTRL_MODE, 0, velocity_m, has_work_num
        )
        command.kp = self._datas.loc["velocity_ctrl_kp"][id]
        command.kd = self._datas.loc["velocity_ctrl_kd"][id]
        return command
