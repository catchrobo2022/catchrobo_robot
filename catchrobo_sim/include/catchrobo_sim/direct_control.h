#pragma once

#include "catchrobo_sim/accel_curve.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/control_result.h"
#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

//#

class DirectControl
{
public:
    DirectControl(){};
    void setRosCmd(const catchrobo_msgs::MyRosCmd &command, const StateStruct &joint_state)
    {
        target_ = command;
    };

    // dt間隔で呼ばれる想定
    void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult &result)
    {
        //// accel_curveを入れると、速度の誤差が大きいため変な動きをする
        command.id = target_.id;
        command.p_des = target_.position;
        command.v_des = target_.velocity;
        command.torque_feed_forward = target_.effort;
        command.kp = target_.kp;
        command.kd = target_.kd;
        result = ControlResult::RUNNING;
    };

private:
    catchrobo_msgs::MyRosCmd target_;
};
