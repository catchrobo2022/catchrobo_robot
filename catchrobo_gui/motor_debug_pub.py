#!/usr/bin/env python

import rospy
from catchrobo_msgs.msg import ErrorCode
import std_msgs

motor_error = [0,1,0]

if __name__=='__main__':
    rospy.init_node('motor_status_')
    pub = rospy.Publisher('error', ErrorCode, queue_size=100)
    rate = rospy.Rate(1)
    msg = ErrorCode()

    count = 0
    while not rospy.is_shutdown():
        if count % 3 == 0:
            msg.id = 0
            msg.error_code = motor_error[0]
        elif count % 3 == 1:
            msg.id = 1
            msg.error_code = motor_error[1]
        elif count % 3 == 2:
            msg.id = 2
            msg.error_code = motor_error[2]    
        pub.publish(msg)
        count += 1
        rate.sleep()
