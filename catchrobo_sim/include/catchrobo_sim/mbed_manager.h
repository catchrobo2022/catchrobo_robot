#pragma once

#include "catchrobo_sim/robot_manager.h"

#ifndef USE_MBED
#include "catchrobo_sim/ros_bridge_sim.h"
#include "catchrobo_sim/motor_driver_bridge_sim.h"
#else
#include "catchrobo_sim/ros_bridge_mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#endif

#include "catchrobo_sim/motor_driver_struct.h"

#include <sensor_msgs/JointState.h>

class MbedManager
{
public:
    MbedManager(){};
    void init(float motor_driver_cmd_dt, int joint_num, char *joint_name[], RosBridge *ros_bridge, MotorDriverBridge *motor_driver_bridge)
    {
        ros_bridge_ = ros_bridge;
        motor_driver_bridge_ = motor_driver_bridge;
        joint_num_ = joint_num;
        // ros_bridge_.init(ros_baudrate, &MbedManager::rosCallback, this);
        robot_manager_.init(motor_driver_cmd_dt, joint_num, joint_name);
    };
    void rosCallback(const catchrobo_msgs::MyRosCmd &command)
    {
        // radius
        robot_manager_.setRosCmd(command);
    };

    void motorDriverCallback(const StateStruct &input)
    {
        // radius
        robot_manager_.setCurrentState(input);
    };

    void mbed2RosTimerCallback()
    {
        // radius
        sensor_msgs::JointState joint_state;
        robot_manager_.getJointState(joint_state);
        ros_bridge_->publishJointState(joint_state);
    };

    void mbed2MotorDriverTimerCallback()
    {
        // radius
        ////[TODO] マジックナンバーを消す
        for (size_t i = 0; i < 3; i++)
        {
            //// ブラシレスモーターへ指示
            bool finished = false;
            ControlStruct cmd;
            robot_manager_.getCmd(i, cmd, finished);

            // ROS_INFO_STREAM(cmd.id);
            // ROS_INFO_STREAM(cmd.v_des);
            motor_driver_bridge_->publish(cmd);
            if (finished)
            {
                ros_bridge_->publishFinishFlag(i);
            }
        }
    };

private:
    RosBridge *ros_bridge_;
    MotorDriverBridge *motor_driver_bridge_;
    RobotManager robot_manager_;
    int joint_num_;
};
