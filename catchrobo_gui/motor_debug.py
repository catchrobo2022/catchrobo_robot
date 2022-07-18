#!/usr/bin/env python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
from catchrobo_msgs.msg import ErrorCode
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from enum import IntEnum
import time

DERAY = 2

class Status(IntEnum):
    NONE=0
    OVER_POSITION=1
    OVER_VELOCITY=2
    OVER_TORQUE=3
    FAR_TARGET_POSITION=4
    COLLISION=5
    FINISH=6

motor_error = [Status.NONE,Status.NONE,Status.NONE,Status.NONE]
start_time = time.time()
motor_time = [0,0,0,0]
motor_current = [0,0,0]

def callback(msg):
    motor_error[msg.id] = msg.error_code
    for i in range(4):
        if motor_error[i] == Status.FINISH:
            motor_time[i] = time.time() 
        else:
            motor_time[i] = 0

def check_motor0(stat):
    if motor_error[0] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    elif motor_error[0] == Status.FINISH:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, Status(motor_error[0]))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[0]))
    return stat
def check_motor1(stat):
    if motor_error[1] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    elif motor_error[1] == Status.FINISH:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, Status(motor_error[1]))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[1]))
    return stat
def check_motor2(stat):
    if motor_error[2] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    elif motor_error[2] == Status.FINISH:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, Status(motor_error[2]))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[2]))
    return stat
def check_motor3(stat):
    if motor_error[3] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    elif motor_error[3] == Status.FINISH:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, Status(motor_error[3]))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[3]))
    return stat

def callback2(msg):
    motor_current[0] = msg.data[4]
    motor_current[1] = msg.data[5]
    motor_current[2] = msg.data[6]

if __name__=='__main__':
    rospy.init_node('motor_status')

    pub0 = rospy.Publisher("/motor0_current", Float32, queue_size=10)
    pub1 = rospy.Publisher("/motor1_current", Float32, queue_size=10)
    pub2 = rospy.Publisher("/motor2_current", Float32, queue_size=10)

    sub = rospy.Subscriber('error', ErrorCode, callback)
    sub_ = rospy.Subscriber('joint_rad', Float32MultiArray, callback2)
    updater = diagnostic_updater.Updater()

    updater.setHardwareID("Motor")
    updater.add("motor0",check_motor0)
    updater.add("motor1",check_motor1)
    updater.add("motor2",check_motor2)
    updater.add("gripper",check_motor3)

    while not rospy.is_shutdown():
        pub0.publish(motor_current[0])
        pub1.publish(motor_current[1])
        pub2.publish(motor_current[2])
        updater.update()
        rospy.sleep(0.1)
        for i in range(4):
            if motor_time[i] != 0:
                if time.time()-motor_time[i] > DERAY:
                    motor_error[i] = Status.NONE
