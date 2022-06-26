#pragma once

#include "catchrobo_sim/define.h"
#include "catchrobo_sim/motor_manager.h"
#include "catchrobo_sim/servo_manager.h"
#include "catchrobo_sim/peg_in_hole_control.h"
#include "catchrobo_sim/enable_manager.h"
#include "motor_driver_bridge/motor_driver_struct.h"

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>

#include <vector>
#include <string>

#define TORQUE_OUTPUT
// #

class RobotManager
{
public:
    RobotManager() : is_peg_in_hole_mode_(false), motor_num_(N_MOTORS)
    {
        for (int i = 0; i < motor_num_; i++)
        {
            motor_manager_[i] = new MotorManager;
        }
        motor_manager_[N_MOTORS] = new ServoManager;

        actuator_num_ = sizeof(motor_manager_) / sizeof(motor_manager_[0]);

        ////[TODO] 面倒なので直打ち
        const int joint_num = JOINT_NUM;
        char *joint_name[JOINT_NUM] = {"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};
        resetJointState(joint_num, joint_name);
        resetJointRad(joint_num);
    }

#ifdef USE_MBED

    ~RobotManager()
    {
        if (joint_state_.position != NULL)
            delete[] joint_state_.position;
        if (joint_state_.velocity != NULL)
            delete[] joint_state_.position;
        if (joint_state_.effort != NULL)
            delete[] joint_state_.position;
        if (joint_rad_.data != NULL)
            delete[] joint_rad_.data;
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

    void resetJointRad(int joint_num)
    {
        int num = joint_num;
#ifdef TORQUE_OUTPUT
        num = num * 2;
#endif
        joint_rad_.data_length = num;
        joint_rad_.data = new float[num];
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
    void resetJointRad(int joint_num)
    {
        int num = joint_num;

#ifdef TORQUE_OUTPUT
        num = num * 2;
#endif
        joint_rad_.data.resize(num);
    }
#endif
    void setPegInHoleCmd(const std_msgs::Bool &input)
    {
        peg_in_hole_control_.setPegInHoleCmd(input, joint_rad_);
    };
    ////本当はMyRosCmdArrayを受け取るのがキレイだが、配列要素数を取得する計算がPCとmbedで変わってしまうため、MyRosCmdで受け取るようにしている。
    void setRosCmd(const catchrobo_msgs::MyRosCmd &command)
    {
        motor_manager_[command.id]->setRosCmd(command);
    };

    // void setPegInHoleCmd(const catchrobo_msgs)

    void getMotorDrivesCommand(ControlStruct (&cmd)[JOINT_NUM], ControlResult::ControlResult (&result)[JOINT_NUM])
    {

        ////もしpeg in holeなら、指示を変える
        if (peg_in_hole_control_.isPegInHoleMode())
        {
            catchrobo_msgs::MyRosCmd ros_cmd[JOINT_NUM];
            for (size_t i = 0; i < motor_num_; i++)
            {
                motor_manager_[i]->getRosCmd(ros_cmd[i]);
            }
            StateStruct z_state;
            motor_manager_[2]->getState(z_state);
            peg_in_hole_control_.getCmd(z_state, ros_cmd, result);
            ////resultはpeg_in_hole_control_で計算してあるので、dummyを使う
            ControlResult::ControlResult dummy[JOINT_NUM];
            for (int i = 0; i < motor_num_; i++)
            {
                motor_manager_[i]->setRosCmd(ros_cmd[i]);
                motor_manager_[i]->getCmd(cmd[i], dummy[i]);
            }
        }
        else
        {
            //// 通常のコントロール
            for (size_t i = 0; i < actuator_num_; i++)
            {
                motor_manager_[i]->getCmd(cmd[i], result[i]);
            }
        }

        ///// cmdのidを上書き。初期値である0になっている場合があるため
        for (size_t i = 0; i < actuator_num_; i++)
        {
            cmd[i].id = i;
        }
    };

    void nextStep(float dt)
    {
        for (size_t i = 0; i < actuator_num_; i++)
        {
            motor_manager_[i]->nextStep(dt);
        }

        peg_in_hole_control_.nextStep(dt);
    }

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

    void getJointRad(std_msgs::Float32MultiArray &joint_state)
    {

        for (size_t i = 0; i < actuator_num_; i++)
        {
            StateStruct state;
            motor_manager_[i]->getState(state);
            joint_rad_.data[i] = state.position;
#ifdef TORQUE_OUTPUT
            joint_rad_.data[i + actuator_num_] = state.torque;
#endif
        }
        joint_state = joint_rad_;
    };

    void disable()
    {
        catchrobo_msgs::MyRosCmd command;
        command.mode = catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE;
        command.kp = 0;
        command.kd = 0;
        command.effort = 0;
        for (size_t i = 0; i < motor_num_; i++)
        {
            /* code */
            command.id = i;
            setRosCmd(command);
        }
    }

private:
    MotorManager *(motor_manager_[JOINT_NUM]);
    sensor_msgs::JointState joint_state_;
    std_msgs::Float32MultiArray joint_rad_;

    int actuator_num_;
    int is_peg_in_hole_mode_;
    const int motor_num_;

    PegInHoleControl peg_in_hole_control_;

    // void independentControl(ControlStruct (&cmd)[JOINT_NUM], ControlResult::ControlResult (&result)[JOINT_NUM])
    // {
    //     for (size_t i = 0; i < actuator_num_; i++)
    //     {
    //         motor_manager_[i]->getCmd(cmd[i], result[i]);
    //     }
    // }

    // void pegInHole(ControlStruct (&cmd)[JOINT_NUM], ControlResult::ControlResult (&result)[JOINT_NUM])
    // {

    //     ////xyz軸のros cmdを変更
    //     catchrobo_msgs::MyRosCmd ros_cmd[JOINT_NUM];
    //     StateStruct z_state;
    //     motor_manager_[2]->getState(z_state);
    //     peg_in_hole_control_.getCmd(z_state, ros_cmd, result);
    //     for (int i = 0; i < 3; i++)
    //     {
    //         motor_manager_[i]->setRosCmd(ros_cmd[i]);
    //     }
    //     ////resultはpeg_in_hole_control_で計算するので、dummyを使う
    //     ControlResult::ControlResult dummy[JOINT_NUM];
    //     independentControl(cmd, dummy);
    // }
};
