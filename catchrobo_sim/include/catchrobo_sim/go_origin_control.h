#pragma once

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

class GoOriginControl
{
public:
    GoOriginControl() : is_arrived_(false){};
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {
        target_ = cmd;
        is_arrived_ = false;
    };

    // dt間隔で呼ばれる想定
    void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult::ControlResult &result)
    {
        //// position, acceralationは無視する
        command.kp = 0;
        command.torque_feed_forward = target_.effort;

        if (is_arrived_)
        {
            result = ControlResult::RUNNING;
            command.kd = 0;
            return;
        }
        else
        {
            //// [WARINIG] 面倒だったので、acceleration_limitをしきい値とみなして実装しちゃってます
            if (state.torque > target_.acceleration_limit)
            {
                result = ControlResult::SET_ORIGIN;
                is_arrived_ = true;
                command.kd = 0;
                return;
            }
            else
            {
                command.kd = target_.kd;
                command.v_des = target_.velocity;
            }
        }
    };

private:
    bool is_arrived_;
    catchrobo_msgs::MyRosCmd target_;
};
