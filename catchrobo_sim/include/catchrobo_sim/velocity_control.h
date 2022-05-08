#pragma once

#include "catchrobo_sim/controller_interface.h"

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h> //ROS_INFO用

class VelocityControl : public ControllerInterface
{
public:
    VelocityControl(){};
    void init(double dt)
    {
        frequency_ = 1.0 / dt;
    }
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const catchrobo_msgs::StateStruct &joint_state)
    {
        target_ = cmd;
    };

    // dt間隔で呼ばれる想定
    void getCmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command)
    {
        packResult2Cmd(state, target_, command);
        NanCheck(except_command, command);
    };

private:
    double frequency_; // =1/dt
    catchrobo_msgs::MyRosCmd target_;

    void packResult2Cmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = target.position;
        cmd.v_des = target.velocity;

        double accel = (target.velocity - state.velocity) * frequency_;
        cmd.i_ff = target.inertia * accel + target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }
};
