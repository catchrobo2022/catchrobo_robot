#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from jagarico.target_jagarico_calculator import TargetJagaricoCalculator
from jagarico.jagarico_database import JagaricoDatabase

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    target=TargetJagaricoCalculator()
    jagarico=JagaricoDatabase()
    jagarico.readCsv("red")
    print("first",target.calcTarget(jagarico))
    jagarico.delete(target.calcTarget(jagarico))
    print("second",target.calcTarget(jagarico))
    rospy.spin()
