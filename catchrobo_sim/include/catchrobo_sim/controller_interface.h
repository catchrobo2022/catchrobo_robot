#pragma once

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>


class ControllerInterface
{
public:
    virtual void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const catchrobo_msgs::StateStruct &joint_state) = 0;
    // dt間隔で呼ばれる想定
    virtual void getCmd(const catchrobo_msgs::ControlStruct &old_cmd, catchrobo_msgs::ControlStruct &cmd) = 0;
};
