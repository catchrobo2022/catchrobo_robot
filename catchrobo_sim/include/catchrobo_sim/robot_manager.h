#pragma once

#include "catchrobo_sim/define.h"
#include "catchrobo_sim/motor_manager.h"
#include "catchrobo_sim/servo_manager.h"
#include "catchrobo_sim/peg_in_hole_control.h"
#include "motor_driver_bridge/motor_driver_struct.h"

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

#include <vector>
#include <string>

// #

class RobotManager
{
public:
    RobotManager() : is_peg_in_hole_mode_(false)
    {
        for (int i = 0; i < N_MOTORS; i++)
        {
            motor_manager_[i] = new MotorManager;
        }
        motor_manager_[N_MOTORS] = new ServoManager;

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

    void init(double dt)
    {
        ////[TODO] 面倒なので直打ち
        char *joint_name[JOINT_NUM] = {"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};
        resetJointState(JOINT_NUM, joint_name);
        for (size_t i = 0; i < actuator_num_; i++)
        {
            motor_manager_[i]->init(dt);
        }

        peg_in_hole_control_.init(dt);
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
        peg_in_hole_control_.setRosCmd(command);
    };

    void getMotorDrivesCommand(ControlStruct (&cmd)[JOINT_NUM], ControlResult (&result)[JOINT_NUM])
    {
        independentControl(cmd, result);

        ////もしpeg in holeなら、指示を変える
        if (peg_in_hole_control_.isPegInHoleMode())
        {
            pegInHole(cmd, result);
        }

        ///// cmdのidを上書き。初期値である0になっている場合があるため
        for (size_t i = 0; i < actuator_num_; i++)
        {
            cmd[i].id = i;
        }
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
    MotorManager *(motor_manager_[JOINT_NUM]);
    sensor_msgs::JointState joint_state_;
    int actuator_num_;
    int is_peg_in_hole_mode_;

    PegInHoleControl peg_in_hole_control_;

    void independentControl(ControlStruct (&cmd)[JOINT_NUM], ControlResult (&result)[JOINT_NUM])
    {
        for (size_t i = 0; i < actuator_num_; i++)
        {
            motor_manager_[i]->getCmd(cmd[i], result[i]);
        }
    }

    void pegInHole(ControlStruct (&cmd)[JOINT_NUM], ControlResult (&result)[JOINT_NUM])
    {

        ////xyz軸のros cmdを変更
        catchrobo_msgs::MyRosCmd ros_cmd[JOINT_NUM];
        StateStruct z_state;
        motor_manager_[2]->getState(z_state);
        peg_in_hole_control_.getCmd(z_state, ros_cmd, result);
        for (int i = 0; i < 3; i++)
        {
            motor_manager_[i]->setRosCmd(ros_cmd[i]);
        }
        ////resultはpeg_in_hole_control_で計算するので、dummyを使う
        ControlResult dummy[JOINT_NUM];
        independentControl(cmd, dummy);
    }
};
