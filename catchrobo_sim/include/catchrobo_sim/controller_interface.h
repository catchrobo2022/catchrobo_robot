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

void NanCheck(const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command)
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