#pragma once

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>
#include <algorithm>
// #

class SafeControl
{
public:
    SafeControl() : alpha_(0), obstacle_avoidance_limit_(0),
                    obstacle_avoidance_enable_(false), obstacle_avoidance_is_min_(false) {}
    void setCBFparams(float alpha)
    {
        alpha_ = alpha;
    }
    void getSafeCmd(const StateStruct &state, const catchrobo_msgs::MyRosCmd &target,
                    const ControlStruct &except_command, ControlStruct &command)
    {
        nanCheck(except_command, command);

        float position_min = 0;
        float position_max = 0;
        calcPositionLimit(target, position_min, position_max);
        ControlBarrierFunctions(position_min, position_max, state, command);
        boundIn(position_min, position_max, target.velocity_limit, command);
        // if (target.id == 2)
        // {
        //     ROS_INFO_STREAM(position_min);
        // }
    }
    void setObstacleInfo(bool enable, bool is_min, float limit)
    {
        obstacle_avoidance_enable_ = enable;
        obstacle_avoidance_is_min_ = is_min;
        obstacle_avoidance_limit_ = limit;
    }

private:
    float alpha_;
    float obstacle_avoidance_limit_;
    bool obstacle_avoidance_enable_;
    bool obstacle_avoidance_is_min_;
    void nanCheck(const ControlStruct &except_command, ControlStruct &command)
    {
        // nanチェック
        if (isnan(command.p_des) || isnan(command.v_des) || isnan(command.torque_feed_forward))
        {
            //異常時
            command = except_command;
        }
    }

    void bound(double min, double max, float &target)
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

    void boundIn(float position_min, float position_max, float velocity_limit, ControlStruct &command)
    {
        //[min, max] にコマンドを入れる
        bound(position_min, position_max, command.p_des);
        bound(-velocity_limit, velocity_limit, command.v_des);
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
        }
    }

    void ControlBarrierFunctions(float position_min, float position_max, const StateStruct &state, ControlStruct &command)
    {
        minPositionCBF(state.position, position_min, alpha_, command.v_des);
        maxPositionCBF(state.position, position_max, alpha_, command.v_des);
    }

    void calcPositionLimit(const catchrobo_msgs::MyRosCmd &target, float &position_min, float &position_max)
    {
        position_min = target.position_min;
        position_max = target.position_max;
        if (obstacle_avoidance_enable_)
        {
            if (obstacle_avoidance_is_min_)
            {
                position_min = std::max(obstacle_avoidance_limit_, position_min);
            }
            else
            {
                position_max = std::min(obstacle_avoidance_limit_, position_max);
            }
        }
    }
};
