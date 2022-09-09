// #define USE_MBED

#include "catchrobo_sim/mbed_manager.h"

#ifndef USE_MBED
#include <ros/ros.h>
#include "catchrobo_sim/ros_bridge_sim.h"
#include "catchrobo_sim/motor_driver_bridge_sim.h"
#include "catchrobo_sim/ticker_bridge_sim.h"
#else
#include <ros.h>
#include "catchrobo_sim/ros_bridge_mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#include "catchrobo_sim/ticker_bridge_mbed.h"

#endif

const int ROS_BAUD_RATE = 115200;
const int CAN_BAUD_RATE = 1000000;

const float MBED2ROS_DT = 0.02;    // 50Hz
const float MBED2MOTOR_DT = 0.002; // 500Hz
const int JOINT_NUM = 4;
char *JOINT_NAME[JOINT_NUM] = {"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};

MbedManager sample_node;

void rosCallback(const catchrobo_msgs::MyRosCmd &command)
{
    sample_node.rosCallback(command);
}

void motorDriverCallback(const StateStruct &input)
{
    // radius
    sample_node.motorDriverCallback(input);
};

void mbed2MotorDriverTimerCallback()
{
    sample_node.mbed2MotorDriverTimerCallback();
}

void mbed2RosTimerCallback()
{
    sample_node.mbed2RosTimerCallback();
}

int main(int argc, char **argv)
{
#ifndef USE_MBED
    ros::init(argc, argv, "mbed_sim");
#endif

    //    sample_node = new MbedManager();
    RosBridge ros_bridge;
    MotorDriverBridge motor_driver_bridge;
    TickerBridge ticker_motor_driver_send;
    TickerBridge ticker_ros_send;

    ros_bridge.init(ROS_BAUD_RATE, rosCallback);
    motor_driver_bridge.init(CAN_BAUD_RATE, motorDriverCallback);
    ticker_motor_driver_send.init(MBED2MOTOR_DT, mbed2MotorDriverTimerCallback);
    ticker_ros_send.init(MBED2ROS_DT, mbed2RosTimerCallback);

    sample_node.init(MBED2MOTOR_DT, JOINT_NUM, JOINT_NAME, &ros_bridge, &motor_driver_bridge);
    ros_bridge.spin();

    return 0;
}