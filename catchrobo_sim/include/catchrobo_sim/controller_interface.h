#pragma once

#include "catchrobo_sim/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

class ControllerInterface
{
public:
    virtual void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state) = 0;
    // dt間隔で呼ばれる想定
    virtual void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &cmd, bool &finished) = 0;
};
