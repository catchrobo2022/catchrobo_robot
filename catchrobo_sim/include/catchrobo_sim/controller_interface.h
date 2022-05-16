#pragma once

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h>

class ControllerInterface
{
public:
    virtual void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const catchrobo_msgs::StateStruct &joint_state) = 0;
    // dt間隔で呼ばれる想定
    virtual void getCmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &cmd) = 0;
};

void nanCheck(const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command)
{
    // nanチェック
    if (std::isfinite(command.p_des) && std::isfinite(command.v_des) && std::isfinite(command.i_ff))
    {
        //正常時
    }
    else
    {
        //異常時
        ROS_INFO_STREAM(command);
        command = except_command;
    }
}

void bound(double min, double max, double &target)
{
    if (target < min)
    {
        target = min;
    }
    if (target > max)
    {
        target = max;
    }
}

void minPositionCBF(double position_now, double position_min, double alpha, double &target_velocity)
{
    ////CBF: b>=0 , b:=position_now-position_min
    ////<=> \dot{b}+alpha(b)>=0
    ////<=> target_velocity >= - alpha(b)

    double b = position_now - position_min;

    double temp = -alpha * b;
    if (target_velocity < temp)
    {
        target_velocity = temp;
    }
}

void limitCheck(const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &command)
{
    bound(target.position_min, target.position_max, command.p_des);
    bound(-target.velocity_limit, target.velocity_limit, command.v_des);
}
