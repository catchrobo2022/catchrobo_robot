#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import threading

import rospy
import rospkg


class Database:
    def __init__(self):
        self._count_key = "exist"
        # self._lock = threading.Lock()

    def generate_file_path(self, csv_name):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_manager")
        config_path = pkg_path + "/config/"
        csv = config_path + csv_name
        return csv

    def save_csv(self, csv_name):
        csv = self.generate_file_path(csv_name)
        self._objects.to_csv(csv, index=True)

    def readCsv(self, csv_name):
        csv = self.generate_file_path(csv_name)
        self._objects = pd.read_csv(csv, index_col=0)

    ## id:行番号 key:列名 の値をvalueに更新する
    def updateState(self, id, key, value):
        # with self._lock:
        self._objects.loc[id, key] = value

    ## id:行番号, key:列名 の値を返す
    def getState(self, id, key):
        # with self._lock:
        return self._objects.loc[id, key]

    ## positionを返す [x,y,z]
    def getPosi(self, id):
        # posi = Point()
        # posi.x = self._objects.loc[id, "x":"z"]
        # posi.y = self._objects.loc[id, "y"]
        # posi.z = self._objects.loc[id, "z"]
        return self._objects.loc[id, "x":"z"]

    ## そのidのexistを返す
    def isExist(self, id):
        return self._objects.loc[id, "exist"]

    ## 行データを返す
    def getObj(self, id):
        if id is None:
            return None
        else:
            return self._objects.loc[id]

    ## exist = falseにする
    def delete(self, id):
        self._objects.loc[id, self._count_key] = False

    ## 共通エリアにオブジェクトが残っていればtrue. 無ければfalse
    def isCommonExist(self):
        group = self._objects.groupby("my_area").sum()
        return group.loc[False, "exist"] > 0

    def getIdNum(self):
        return len(self._objects)

    # key:列名の値がvalueであるオブジェクトの数を返す
    def count(self, key, value):
        exist = self._objects[key]
        return np.sum(exist == value)

    def getColumn(self, key):
        return self._objects[key]
