#!/usr/bin/env python

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
import diagnostic_msgs
from catchrobo_msgs.msg import ErrorCode
import std_msgs
from enum import IntEnum

class Status(IntEnum):
    NONE=0
    OVER_POSITION=1
    OVER_VELOCITY=2
    OVER_TORQUE=3
    FAR_TARGET_POSITION=4
    COLLISION=5

motor_error = [Status.NONE,Status.NONE,Status.NONE]

def callback(msg):
    motor_error[msg.id] = msg.error_code

def check_motor0(stat):
    if motor_error[0] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[0]))
    return stat
def check_motor1(stat):
    if motor_error[1] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[1]))
    return stat
def check_motor2(stat):
    if motor_error[2] == Status.NONE:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, Status(motor_error[2]))
    return stat

if __name__=='__main__':
    rospy.init_node('motor_status')

    sub = rospy.Subscriber('error', ErrorCode, callback)
    updater = diagnostic_updater.Updater()

    updater.setHardwareID("Motor")
    updater.add("motor0",check_motor0)
    updater.add("motor1",check_motor1)
    updater.add("motor2",check_motor2)

    while not rospy.is_shutdown():
        updater.update()
        rospy.sleep(0.1)
