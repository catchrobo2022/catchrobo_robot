#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.rad_transform import RadTransform

import rospy
from sensor_msgs.msg import JointState


class Radius2Meter:
    def __init__(self):
        self._transform = RadTransform()

        self._pub = rospy.Publisher("/my_joint_state", JointState, queue_size=1)
        rospy.Subscriber("/joint_state_rad", JointState, self.callback)

    def callback(self, msg):
        meter_msg = msg
        positions = [0] * len(msg.position)
        velocities = [0] * len(msg.velocity)
        # print(msg)
        for i, (theta, velocity) in enumerate(zip(msg.position, msg.velocity)):
            posi = self._transform.rad2rviz_joint_state(i, theta)
            vel = self._transform.rad2rviz_joint_state(i, velocity)
            positions[i] = posi
            velocities[i] = vel
        meter_msg.position = positions
        meter_msg.velocity = velocities
        # print(meter_msg)

        self._pub.publish(meter_msg)


if __name__ == "__main__":
    rospy.init_node("radius2meter")
    node = Radius2Meter()
    rospy.spin()
