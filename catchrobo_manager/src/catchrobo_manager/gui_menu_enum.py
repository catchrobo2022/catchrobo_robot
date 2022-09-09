#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import IntEnum, auto


class GuiMenu(IntEnum):
    NONE = 0
    ORIGIN = auto()
    CALIBRATION = auto()
    POINT1 = auto()
    POINT2 = auto()
    POINT3 = auto()
    POINT4 = auto()
    INIT = auto()
    START = auto()
