#pragma once

#include "catchrobo_sim/motor_manager.h"
#include "catchrobo_sim/servo_manager.h"
#include "catchrobo_sim/motor_driver_struct.h"

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

#include <vector>
#include <string>

// #

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

        actuator_num_ = sizeof(motor_manager_) / sizeof(motor_manager_[0]);
        // ROS_INFO_STREAM("actuator_num_" << actuator_num_);
        //
        //        joint_state_.name = std::vector<std::string>{"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};
        //
        //        int joint_num = joint_state_.name.size();
        //        joint_state_.position = std::vector<double>(joint_num, 0);
        //        joint_state_.velocity = std::vector<double>(joint_num, 0);
        //        joint_state_.effort = std::vector<double>(joint_num, 0);
    };

    ////本当はjoint_stateの初期化をしたいが、PCとmbedで実装が異なるため引数にしている
    void init(double dt, int joint_num, char *joint_name[])
    {
        resetJointState(joint_num, joint_name);
        for (size_t i = 0; i < actuator_num_; i++)
        {
            motor_manager_[i]->init(dt);
        }
    };

#ifdef USE_MBED

    ~RobotManager()
    {
        if (joint_state_.position != NULL)
            delete[] joint_state_.position;
        if (joint_state_.velocity != NULL)
            delete[] joint_state_.position;
        if (joint_state_.effort != NULL)
            delete[] joint_state_.position;
    }
    void resetJointState(int joint_num, char *joint_name[])
    {

        joint_state_.name_length = joint_num;
        joint_state_.position_length = joint_num;
        joint_state_.velocity_length = joint_num;
        joint_state_.effort_length = joint_num;

        joint_state_.name = joint_name;
        joint_state_.position = new double[joint_num]; //(double *)malloc(sizeof(double)*joint_num)
        joint_state_.velocity = new double[joint_num];
        joint_state_.effort = new double[joint_num];
    }
#endif

#ifndef USE_MBED
    void resetJointState(int joint_num, char *joint_name[])
    {
        std::vector<std::string> name(joint_num);
        for (size_t i = 0; i < joint_num; i++)
        {
            name[i] = joint_name[i];
        }
        joint_state_.name = name;
        joint_state_.position = std::vector<double>(joint_num, 0);
        joint_state_.velocity = std::vector<double>(joint_num, 0);
        joint_state_.effort = std::vector<double>(joint_num, 0);
    }
#endif

    ////本当はMyRosCmdArrayを受け取るのがキレイだが、配列要素数を取得する計算がPCとmbedで変わってしまうため、MyRosCmdで受け取るようにしている。
    void setRosCmd(const catchrobo_msgs::MyRosCmd &command)
    {
        motor_manager_[command.id]->setRosCmd(command);
    };

    void getCmd(int id, ControlStruct &cmd, bool &finished)
    {
        // ROS_INFO_STREAM("getCmd in robot_manager");
        // ROS_INFO_STREAM(id);
        motor_manager_[id]->getCmd(cmd, finished);
        cmd.id = id;
        // ROS_INFO_STREAM(cmd);
    };
    void setCurrentState(const StateStruct &state)
    {
        motor_manager_[state.id]->setCurrentState(state);
    };
    void getJointState(sensor_msgs::JointState &joint_state)
    {

        for (size_t i = 0; i < actuator_num_; i++)
        {
            StateStruct state;
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
    int actuator_num_;
};
