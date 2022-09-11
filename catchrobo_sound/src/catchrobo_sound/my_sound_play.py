# -*- coding: utf-8 -*-

import rospkg

import subprocess
import os


class SoundClient:
    def __init__(self) -> None:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_sound")
        self._sound_path = pkg_path + "/sound/"

    def play(self, t):
        PATH = os.path.dirname(__file__)
        mpg123 = ["mpg123", "-q", self._sound_path + t]
        self._wr = subprocess.Popen(mpg123)

    def wait(self):
        self._wr.wait()

    def __delete__(self):
        self._wr.terminate()
