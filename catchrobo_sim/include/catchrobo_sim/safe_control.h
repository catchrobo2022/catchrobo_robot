#pragma once

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h>

class SafeControl
{
public:
    void setCBFparams(double alpha)
    {
        alpha_ = alpha;
    }
    void getSafeCmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::MyRosCmd &target,
                    const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command)
    {

        nanCheck(except_command, command);
        ControlBarrierFunctions(state, target, command);
        boundIn(target, command);
    }

private:
    double alpha_;
    void nanCheck(const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command)
    {
        // nanチェック
        if (std::isfinite(command.p_des) && std::isfinite(command.v_des) && std::isfinite(command.torque_feed_forward))
        {
            //正常時
        }
        else
        {
            //異常時
            // ROS_INFO_STREAM(command);
            command = except_command;
        }
    }

    void bound(double min, double max, double &target)
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

    void boundIn(const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &command)
    {
        //[min, max] にコマンドを入れる
        bound(target.position_min, target.position_max, command.p_des);
        bound(-target.velocity_limit, target.velocity_limit, command.v_des);
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
            // ROS_INFO_STREAM("min CBF");
        }
    }

    void maxPositionCBF(double position_now, double position_max, double alpha, double &target_velocity)
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

    void ControlBarrierFunctions(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &command)
    {
        minPositionCBF(state.position, target.position_min, alpha_, command.v_des);
        maxPositionCBF(state.position, target.position_max, alpha_, command.v_des);
    }
};
