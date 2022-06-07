#pragma once

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

// #

class SafeControl
{
public:
    void setCBFparams(double alpha)
    {
        alpha_ = alpha;
    }
    void getSafeCmd(const StateStruct &state, const catchrobo_msgs::MyRosCmd &target,
                    const ControlStruct &except_command, ControlStruct &command)
    {

        nanCheck(except_command, command);
        ControlBarrierFunctions(state, target, command);
        boundIn(target, command);
    }

private:
    double alpha_;
    void nanCheck(const ControlStruct &except_command, ControlStruct &command)
    {
        // nanチェック
        if (isnan(command.p_des) || isnan(command.v_des) || isnan(command.torque_feed_forward))
        {
            //異常時
            // ROS_INFO_STREAM(command);
            command = except_command;
        }
    }

    void bound(double min, double max, float &target)
    {
        if (target < min)
        {
            target = min;
            // ROS_INFO_STREAM("min");
        }
        if (target > max)
        {
            target = max;
            // ROS_INFO_STREAM("max");
        }
    }

    void boundIn(const catchrobo_msgs::MyRosCmd &target, ControlStruct &command)
    {
        //[min, max] にコマンドを入れる
        bound(target.position_min, target.position_max, command.p_des);
        bound(-target.velocity_limit, target.velocity_limit, command.v_des);
    }

    void minPositionCBF(double position_now, double position_min, double alpha, float &target_velocity)
    {
        ////CBF: b>=0 , b:=position_now-position_min
        ////<=> \dot{b}+alpha(b)>=0
        ////<=> target_velocity >= - alpha(b)
        double b = position_now - position_min;

        double temp = -alpha * b;
        if (target_velocity < temp)
        {
            target_velocity = temp;
            // ROS_INFO_STREAM("min CBF");
        }
    }

    void maxPositionCBF(double position_now, double position_max, double alpha, float &target_velocity)
    {
        ////CBF: b>=0 , b:=position_max - position_now
        ////<=> \dot{b}+alpha(b)>=0
        ////<=>  alpha(b) >= target_velocity
        double b = position_max - position_now;

        double temp = alpha * b;
        if (target_velocity > temp)
        {
            target_velocity = temp;
            // ROS_INFO_STREAM("max CBF");
        }
    }

    void ControlBarrierFunctions(const StateStruct &state, const catchrobo_msgs::MyRosCmd &target, ControlStruct &command)
    {
        minPositionCBF(state.position, target.position_min, alpha_, command.v_des);
        maxPositionCBF(state.position, target.position_max, alpha_, command.v_des);
    }
};
