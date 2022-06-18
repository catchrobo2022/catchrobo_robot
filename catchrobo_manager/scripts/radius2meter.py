#!/usr/bin/env python
# -*- coding: utf-8 -*-

from catchrobo_driver.rad_transform import RadTransform

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class Radius2Meter:
    def __init__(self):
        self._transform = RadTransform()
        self._meter_msg = JointState()
        self._meter_msg.name = [
            "arm/joint1",
            "arm/joint2",
            "arm/joint3",
            "gripper/joint1",
        ]
        self._meter_msg.position = [0] * len(self._meter_msg.name)
        self._meter_msg.velocity = [0] * len(self._meter_msg.name)
        self._meter_msg.effort = [0] * len(self._meter_msg.name)
        self._pub = rospy.Publisher("/my_joint_state", JointState, queue_size=1)
        rospy.Subscriber("/joint_rad", Float32MultiArray, self.callback)

    def callback(self, msg):

        positions = [0] * len(msg.data)
        velocities = [0] * len(msg.data)
        effort = [0] * len(msg.data)
        # print(msg)
        for i, theta in enumerate(msg.data):
            posi = self._transform.rad2rviz_joint_state(i, theta)
            # vel = self._transform.rad2rviz_joint_state(i, velocity)
            positions[i] = posi
            # velocities[i] = vel
        self._meter_msg.position = positions
        # meter_msg.velocity = velocities
        # print(meter_msg)

        self._pub.publish(self._meter_msg)


if __name__ == "__main__":
    rospy.init_node("radius2meter")
    node = Radius2Meter()
    rospy.spin()
