#pragma once

#include "catchrobo_sim/motor_manager.h"
#include "catchrobo_sim/servo_manager.h"

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/StateStruct.h>

#include <vector>

#include <ros/ros.h> //ROS_INFO用

class RobotManager
{
public:
    RobotManager()
    {
        for (int i = 0; i < 3; i++)
        {
            motor_manager_[i] = new MotorManager;
        }
        motor_manager_[3] = new ServoManager;

        joint_state_.name = std::vector<std::string>{"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};

        int joint_num = joint_state_.name.size();
        joint_state_.position = std::vector<double>(joint_num, 0);
        joint_state_.velocity = std::vector<double>(joint_num, 0);
        joint_state_.effort = std::vector<double>(joint_num, 0);
    };
    void init(double dt)
    {
        for (size_t i = 0; i < joint_state_.name.size(); i++)
        {
            motor_manager_[i]->init(dt);
        }
    };

    void setRosCmd(const catchrobo_msgs::MyRosCmdArray &input)
    {
        for (const catchrobo_msgs::MyRosCmd &command : input.command_array)
        {
            motor_manager_[command.id]->setRosCmd(command);
        }
    };
    void getCmd(int id, catchrobo_msgs::ControlStruct &cmd, bool &finished)
    {
        // ROS_INFO_STREAM("getCmd in robot_manager");
        // ROS_INFO_STREAM(id);
        motor_manager_[id]->getCmd(cmd, finished);
        cmd.id = id;

        // ROS_INFO_STREAM(cmd);
    };
    void setCurrentState(const catchrobo_msgs::StateStruct &state)
    {
        motor_manager_[state.id]->setCurrentState(state);
    };
    void getJointState(sensor_msgs::JointState &joint_state)
    {

        for (size_t i = 0; i < joint_state_.name.size(); i++)
        {
            catchrobo_msgs::StateStruct state;
            motor_manager_[i]->getState(state);
            joint_state_.position[i] = state.position;
            joint_state_.velocity[i] = state.velocity;
            joint_state_.effort[i] = state.torque;
        }
        joint_state = joint_state_;
    };

private:
    MotorManager *(motor_manager_[4]);
    sensor_msgs::JointState joint_state_;
};
