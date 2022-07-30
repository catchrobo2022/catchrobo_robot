#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from catchrobo_manager.rad_transform import RadTransform
from catchrobo_manager.robot_transform import WorldRobotTransform

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


class Radius2Meter:
    def __init__(self):
        self._transform = RadTransform()
        self._meter_msg = JointState()
        name_space = "radius2meter/"
        self._meter_msg.name = rospy.get_param(name_space + "joint_names")

        self.WORLD_FRAME = rospy.get_param(name_space + "world_frame")
        robot_origin_m = rospy.get_param("robot/robot_origin_m")

        self.FIELD = rospy.get_param("field")
        self._world_transform = WorldRobotTransform(self.FIELD, robot_origin_m)

        world_position_topic = rospy.get_param(name_space + "world_position_topic")
        self._pub_world_position = rospy.Publisher(
            world_position_topic, PoseStamped, queue_size=1
        )

        my_joint_state = rospy.get_param("joint_state_publisher/source_list")[0]
        self._pub = rospy.Publisher(my_joint_state, JointState, queue_size=1)
        rospy.Subscriber("joint_rad", Float32MultiArray, self.callback)

        self._t = rospy.Time.now()

    def pub2rviz(self, msg: Float32MultiArray):
        current_t = rospy.Time.now()
        dt = current_t - self._t
        self._t = current_t

        joint_num = len(self._meter_msg.name)
        positions = [0] * joint_num
        velocities = [0] * joint_num
        effort = [0] * joint_num
        # print(msg)
        for i in range(joint_num):
            theta = msg.data[i]
            posi = self._transform.rad2rviz_joint_state(i, theta)
            # vel = self._transform.rad2rviz_joint_state(i, velocity)
            positions[i] = posi
            velocities[i] = (posi - self._meter_msg.position[i]) / dt.to_sec()
            effort[i] = msg.data[
                i + joint_num
            ]  # (velocities[i] - self._meter_msg.velocity[i]) / self._dt
            # velocities[i] = vel

        # for i in range(3):
        #     positions[i] *= -1
        #     velocities[i] *= -1
        #     effort[i] *= -1
        self._meter_msg.position = positions
        self._meter_msg.velocity = velocities
        self._meter_msg.effort = effort
        self._meter_msg.header.stamp = current_t
        # meter_msg.velocity = velocities
        # print(meter_msg)

        self._pub.publish(self._meter_msg)

    def pub_world_position(self, msg: Float32MultiArray):

        robot_m = [0] * 3
        for i in range(3):
            theta = msg.data[i]
            robot_m[i] = self._transform.robot_m2rad(i, theta)
        world_m = self._world_transform.robot2world(robot_m)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.WORLD_FRAME
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.orientation.w = 1

        pose_stamped.pose.position.x = world_m[0]
        pose_stamped.pose.position.y = world_m[1]
        pose_stamped.pose.position.z = world_m[2]
        pose_stamped.pose.orientation.w = 1
        self._pub_world_position.publish(pose_stamped)

    def callback(self, msg):
        self.pub2rviz(msg)
        self.pub_world_position(msg)


if __name__ == "__main__":
    rospy.init_node("radius2meter")
    node = Radius2Meter()
    rospy.spin()
