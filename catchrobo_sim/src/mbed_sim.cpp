//  #define USE_MBED

#ifdef USE_MBED
#include "mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#include "catchrobo_sim/ros_bridge_mbed.h"
#else
#include <ros/ros.h>
#include "sim_only/motor_driver_bridge_sim.h"
#include "sim_only/ros_bridge_sim.h"
#include "sim_only/ticker_sim.h"
#endif
#include <std_msgs/Float32MultiArray.h>
#include "catchrobo_sim/robot_manager.h"

const float MBED2ROS_DT = 0.01;    // 10Hz
const float MBED2MOTOR_DT = 0.002; // 500Hz
const int SERIAL_BAUD_RATE = 115200;

MotorDriverBridge motor_driver_bridge;
RosBridge ros_bridge;
RobotManager robot_manager;
EnableManager enable_manager;

void enableAll(bool is_enable)
{
    for (int i = 0; i < JOINT_NUM; i++)
    {
        motor_driver_bridge.enableMotor(i, is_enable);
    }
}

void pegInHoleCallback(const std_msgs::Bool &input)
{
    robot_manager.setPegInHoleCmd(input);
}

void motorDriverCallback(const StateStruct &input)
{
    robot_manager.setCurrentState(input);
};

void rosCallback(const catchrobo_msgs::MyRosCmd &command)
{
    robot_manager.setRosCmd(command);
}

void mbed2MotorDriverTimerCallback()
{
    //// enable check
    catchrobo_msgs::ErrorCode error;
    sensor_msgs::JointState joint_state;
    robot_manager.getJointState(joint_state);
    enable_manager.check(joint_state, error);

    //// errorならdisable指示およびerror のpublish
    if (error.error_code != catchrobo_msgs::ErrorCode::NONE)
    {
        enableAll(false);
        enable_manager.setCurrentEnable(false);
        ros_bridge.publishError(error);
    }

    //// 初期値は全て0 -> 脱力
    // ROS_INFO_STREAM(enable_manager.getEnable());
    if (enable_manager.getEnable()) //// enableならtをすすめる
    {
        robot_manager.nextStep(MBED2MOTOR_DT);
    }
    else //// disableなら脱力指示
    {
        robot_manager.disable();
        // ROS_INFO_STREAM("disable");
    }
    ControlStruct control[JOINT_NUM] = {};
    ControlResult::ControlResult result[JOINT_NUM] = {};
    robot_manager.getMotorDrivesCommand(control, result);
    //// update target value
    for (int i = 0; i < JOINT_NUM; i++)
    {
        motor_driver_bridge.publish(control[i]);
        if (result[i] == ControlResult::FINISH)
        {
            ros_bridge.publishFinishFlag(i);
        }
    }
    // switch (result[i])
    // {
    // case ControlResult::RUNNING:
    //     break;
    // case ControlResult::FINISH:
    //     ros_bridge.publishFinishFlag(i);
    //     break;
    // // case ControlResult::SET_ORIGIN:
    // //     motor_driver_bridge.setOrigin(i);
    // //     ros_bridge.publishFinishFlag(i);
    // //     break;
    // default:
    //     break;
    // }
}

void mbed2RosTimerCallback()
{
    //// radius
    // sensor_msgs::JointState joint_state;
    // robot_manager.getJointState(joint_state);

    std_msgs::Float32MultiArray joint_state;
    robot_manager.getJointRad(joint_state);
    ros_bridge.publishJointState(joint_state);
};

// void enableCallback(const std_msgs::Bool &input)
// {
//     if (input.data)
//     {
//         for (int i = 0; i < N_MOTORS; i++)
//         {
//             motor_driver_bridge.enableMotor(i);
//         }
//     }
//     else
//     {
//         for (int i = 0; i < N_MOTORS; i++)
//         {
//             motor_driver_bridge.disableMotor(i);
//         }
//     }
// }
void enableCallback(const catchrobo_msgs::EnableCmd &input)
{
    enable_manager.setCmd(input);
    enableAll(input.is_enable);
    enable_manager.setCurrentEnable(input.is_enable);
}

int main(int argc, char **argv)
{
#ifndef USE_MBED
    ros::init(argc, argv, "mbed_sim");
    ros::NodeHandle nh("");
    motor_driver_bridge.setNodeHandlePtr(&nh);
    ros_bridge.setNodeHandlePtr(&nh);
#endif

    ros_bridge.init(SERIAL_BAUD_RATE, rosCallback, enableCallback, pegInHoleCallback);
    motor_driver_bridge.init(motorDriverCallback);

    //// 初期値を暫定原点にする。後にROS指示で原点だしを行う
    for (size_t i = 0; i < N_MOTORS; i++)
    {
        motor_driver_bridge.setOrigin(i);
    }

    //// motor driverへの指示開始
    Ticker ticker_motor_driver_send;
    ticker_motor_driver_send.attach(&mbed2MotorDriverTimerCallback, MBED2MOTOR_DT);

    //// ros へのフィードバック開始
    Ticker ticker;
    ticker.attach(&mbed2RosTimerCallback, MBED2ROS_DT);
    ros_bridge.spin();
}