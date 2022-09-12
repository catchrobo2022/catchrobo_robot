#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int8, Float32, Time

if __name__ == "__main__":
    rospy.init_node("time_test_node", anonymous=True)
    rospy.loginfo("Manual Mode!!")
    pub_manual_time=rospy.Publisher("manual_time", Float32, queue_size=1)


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        start=rospy.get_time()
        rospy.sleep(2.0)
        finish=rospy.get_time()
        total=Float32()
        total.data=finish-start
        rospy.loginfo(total)

        # total_int=int(total.data)
        # rospy.loginfo(total_int)
        # rospy.loginfo(type(total))
        
        pub_manual_time.publish(total)
        rate.sleep()
