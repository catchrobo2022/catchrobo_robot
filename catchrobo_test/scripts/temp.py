#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


points = np.zeros((4, 2))
points[0] = [4, 8]
points[1] = np.asarray([12, 16])
print(np.asarray(points))


print(np.average(points, axis=0))
