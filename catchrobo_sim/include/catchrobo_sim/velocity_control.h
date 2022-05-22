#pragma once

#include "catchrobo_sim/controller_interface.h"
#include "catchrobo_sim/accel_curve.h"
#include "catchrobo_sim/safe_control.h"

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h> //ROS_INFO用

class VelocityControl : public ControllerInterface
{
public:
    VelocityControl(){};
    void init(double dt, SafeControl &safe_control)
    {
        dt_ = dt;
        safe_control_ = safe_control;
    }
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const catchrobo_msgs::StateStruct &joint_state)
    {
        target_ = cmd;

        // accel_curve_.reset(cmd.jerk_limit, cmd.acceleration_limit, joint_state.velocity, cmd.velocity);
    };

    // dt間隔で呼ばれる想定
    void getCmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command, bool &finished)
    {
        packResult2Cmd(state, target_, command);
        safe_control_.getSafeCmd(state, target_, except_command, command);
        finished = false;
    };

private:
    double dt_;
    double frequency_; // =1/dt
    catchrobo_msgs::MyRosCmd target_;
    ctrl::AccelCurve accel_curve_;
    SafeControl safe_control_;

    void packResult2Cmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = target.position;
        cmd.v_des = target.velocity;

        double accel = (target.velocity - state.velocity) / dt_;
        if (accel > target.acceleration_limit)
        {
            accel = target.acceleration_limit;
            cmd.v_des = state.velocity + accel * dt_;
        }
        if (accel < -target.acceleration_limit)
        {
            accel = -target.acceleration_limit;
            cmd.v_des = state.velocity + accel * dt_;
        }

        cmd.torque_feed_forward = target.mass * accel + target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }
};
