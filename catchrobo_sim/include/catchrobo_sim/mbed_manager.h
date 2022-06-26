#pragma once

#include "catchrobo_sim/robot_manager.h"

#ifndef USE_MBED
#include "catchrobo_sim/ros_bridge_sim.h"
#include "catchrobo_sim/motor_driver_bridge_sim.h"
#include "catchrobo_sim/ticker_bridge_sim.h"
#else
#include "catchrobo_sim/ros_bridge_mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#include "catchrobo_sim/ticker_bridge_mbed.h"
#endif

#include "catchrobo_sim/motor_driver_struct.h"

#include <sensor_msgs/JointState.h>

class MbedManager
{
public:
    MbedManager(){};
    void init(int ros_baudrate, int can_baudrate, double mbed2ros_dt, double motor_driver_cmd_dt, int joint_num, char *joint_name[])
    {
        joint_num_ = joint_num;
        ros_bridge_.init(ros_baudrate, &MbedManager::rosCallback, this);
        motor_driver_bridge_.init(can_baudrate, &MbedManager::motorDriverCallback, this);
        ticker_ros_send_.init(mbed2ros_dt, &MbedManager::mbed2RosTimerCallback, this);
        ticker_motor_driver_send_.init(motor_driver_cmd_dt, &MbedManager::mbed2MotorDriverTimerCallback, this);
        robot_manager_.init(motor_driver_cmd_dt, joint_num, joint_name);
    };
    void spin()
    {
        ros_bridge_.spin();
    }

private:
    RosBridge<MbedManager> ros_bridge_;
    MotorDriverBridge<MbedManager> motor_driver_bridge_;

    TickerBridge<MbedManager> ticker_motor_driver_send_;
    TickerBridge<MbedManager> ticker_ros_send_;
    RobotManager robot_manager_;

    int joint_num_;

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
        ros_bridge_.publishJointState(joint_state);
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
            motor_driver_bridge_.publish(cmd);
            if (finished)
            {
                ros_bridge_.publishFinishFlag(i);
            }
        }
    };
};
