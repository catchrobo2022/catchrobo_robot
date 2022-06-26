// #define USE_MBED
#define USE_SIM

#include <ros/ros.h>
#include "catchrobo_sim/mbed_manager.h"

const int ROS_BAUD_RATE = 115200;
const int CAN_BAUD_RATE = 1000000;

const float MBED2ROS_DT = 0.02;    // 50Hz
const float MBED2MOTOR_DT = 0.002; // 500Hz
const int JOINT_NUM = 4;
char *JOINT_NAME[JOINT_NUM] = {"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};

int main(int argc, char **argv)
{
#ifndef USE_MBED
    ros::init(argc, argv, "mbed_sim");
#endif
    MbedManager sample_node;
    sample_node.init(ROS_BAUD_RATE, CAN_BAUD_RATE, MBED2ROS_DT, MBED2MOTOR_DT, JOINT_NUM, JOINT_NAME);
    sample_node.spin();
    return 0;
}