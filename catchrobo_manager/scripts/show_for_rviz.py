#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from catchrobo_manager.rad_transform import RadTransform
from catchrobo_manager.radius2meter import Radius2Meter

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


# class Radius2Meter:
#     def __init__(self):
#         self._transform = RadTransform()
#         self._meter_msg = JointState()
#         self._meter_msg.name = rospy.get_param("radius2meter/joint_names")
#         my_joint_state = rospy.get_param("joint_state_publisher/source_list")[0]
#         self._pub = rospy.Publisher(my_joint_state, JointState, queue_size=1)
#         self._meter_msg.position = [0] * len(self._meter_msg.name)
#         self._meter_msg.velocity = [0] * len(self._meter_msg.name)
#         self._meter_msg.effort = [0] * len(self._meter_msg.name)
#         rospy.Subscriber("joint_rad", Float32MultiArray, self.callback)

#         self._t = rospy.Time.now()

#     def pub2rviz(self, msg: Float32MultiArray):
#         current_t = rospy.Time.now()
#         dt = current_t - self._t
#         self._t = current_t

#         joint_num = len(self._meter_msg.name)
#         positions = [0] * joint_num
#         velocities = [0] * joint_num
#         effort = [0] * joint_num
#         # print(msg)
#         for i in range(joint_num):
#             theta = msg.data[i]
#             posi = self._transform.rad2rviz_joint_state(i, theta)
#             # vel = self._transform.rad2rviz_joint_state(i, velocity)
#             positions[i] = posi
#             velocities[i] = (posi - self._meter_msg.position[i]) / dt.to_sec()
#             effort[i] = msg.data[
#                 i + joint_num
#             ]  # (velocities[i] - self._meter_msg.velocity[i]) / self._dt
#             # velocities[i] = vel

#         # for i in range(3):
#         #     positions[i] *= -1
#         #     velocities[i] *= -1
#         #     effort[i] *= -1
#         self._meter_msg.position = positions
#         self._meter_msg.velocity = velocities
#         self._meter_msg.effort = effort
#         self._meter_msg.header.stamp = current_t
#         # meter_msg.velocity = velocities
#         # print(meter_msg)

#         self._pub.publish(self._meter_msg)

#     def callback(self, msg):
#         self.pub2rviz(msg)


if __name__ == "__main__":
    rospy.init_node("radius2meter")
    node = Radius2Meter()
    rospy.spin()
