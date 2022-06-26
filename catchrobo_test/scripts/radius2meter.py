#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState
import math

class Radius2Meter:
    def __init__(self):
        self._radius = [0.002 * 54/(2*math.pi), 0.002 * 54/(2*math.pi), 0.002 * 54/(2*math.pi)]
        self.pub_ = rospy.Publisher("/my_joint_state", JointState, queue_size=1)
        rospy.Subscriber("/joint_state_radius", JointState, self.callback)
    def callback(self, msg):
        for i, (theta, velocity, radius) in enumerate(zip(msg.position, msg.velocity, self._radius)):
            msg.position[i] = theta * radius
            msg.velocity[i] = velocity * radius
        


if __name__ == "__main__":
    rospy.init_node("radius2meter")
    node = Radius2Meter()
    rospy.spin()
