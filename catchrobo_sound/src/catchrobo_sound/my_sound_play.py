# -*- coding: utf-8 -*-

import rospkg

import subprocess
import os


class SoundClient:
    def __init__(self) -> None:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("catchrobo_sound")
        self._sound_path = pkg_path + "/sound/"
        self._wr = None

    def play(self, t):
        PATH = os.path.dirname(__file__)
        mpg123 = ["mpg123", "-q", self._sound_path + t]
        self._wr = subprocess.Popen(mpg123)

    def wait(self):
        self._wr.wait()

    def stop(self):
        if self._wr is None:
            return
        self._wr.terminate()
