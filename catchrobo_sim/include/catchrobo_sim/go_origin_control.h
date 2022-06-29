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
    void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult::ControlResult &result, float &offset)
    {

        //// 現在値にホールド
        //// [WARINIG] 面倒だったので、positionを原点出し時に到達する位置としています
        command.p_des = target_.position;
        command.v_des = 0;
        command.kp = target_.kp;
        command.kd = target_.kd;
        command.torque_feed_forward = target_.effort;
        if (is_arrived_)
        {
            result = ControlResult::RUNNING;
            return;
        }
        else
        {
            //// [WARINIG] 面倒だったので、acceleration_limitをしきい値とみなして実装しちゃってます
            if (fabs(state.torque) > target_.acceleration_limit)
            {
                result = ControlResult::FINISH;
                offset = state.position - target_.position;
                is_arrived_ = true;
                return;
            }
            else
            {
                //// 速度制御に変更
                command.kp = 0;
                command.v_des = target_.velocity;
            }
        }
    };

private:
    bool is_arrived_;
    catchrobo_msgs::MyRosCmd target_;
};
