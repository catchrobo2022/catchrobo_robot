#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8


mode = [0]
def callback(msg):
    mode[0] = msg.data

if __name__=='__main__':
    rospy.init_node("marker_pub")

    pub = rospy.Publisher("arrow_pub", Marker, queue_size = 10)
    sub = rospy.Subscriber("arrow_sub", Int8, callback)

    marker_data = Marker()
    marker_data.header.frame_id = "base/base_link"
    marker_data.header.stamp = rospy.Time.now()

    marker_data.ns = "basic_shapes"
    marker_data.id = 0

    marker_data.action = Marker.ADD

    while not rospy.is_shutdown():

        if mode[0] == 1:
            marker_data.pose.position.x = 1.225
            marker_data.pose.position.y = 0.875
        elif mode[0] == 2:
            marker_data.pose.position.x = 1.425
            marker_data.pose.position.y = 0.875
        elif mode[0] == 3:
            marker_data.pose.position.x = 1.225
            marker_data.pose.position.y = 1.375
        elif mode[0] == 4:
            marker_data.pose.position.x = 1.225
            marker_data.pose.position.y = -0.875
        elif mode[0] == 5:
            marker_data.pose.position.x = 1.425
            marker_data.pose.position.y = -0.875
        elif mode[0] == 6:
            marker_data.pose.position.x = 1.225
            marker_data.pose.position.y = -1.375
        else:
            marker_data.pose.position.x = 0
            marker_data.pose.position.y = 0
        marker_data.pose.position.z = 0.5

        marker_data.pose.orientation.x=-1.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=1.0
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 1.0
        marker_data.color.b = 0.0
        marker_data.color.a = 0.9

        if mode[0] == 0:
            marker_data.scale.x = 0
            marker_data.scale.y = 0
            marker_data.scale.z = 0
        else:
            marker_data.scale.x = 0.5
            marker_data.scale.y = 0.05
            marker_data.scale.z = 0.05

        marker_data.lifetime = rospy.Duration()

        marker_data.type = 0

        pub.publish(marker_data)

        rospy.sleep(0.1)
