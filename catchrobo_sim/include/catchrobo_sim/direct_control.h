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
    void init(double dt, SafeControl &safe_control)
    {
        safe_control_ = safe_control;
    }
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {
        target_ = cmd;

        // accel_curve_.reset(cmd.jerk_limit, cmd.acceleration_limit, joint_state.velocity, cmd.velocity);
    };

    // dt間隔で呼ばれる想定
    void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult &result)
    {
        packResult2Cmd(target_, command);
        safe_control_.getSafeCmd(state, target_, except_command, command);
        result = ControlResult::RUNNING;
    };

private:
    catchrobo_msgs::MyRosCmd target_;
    ctrl::AccelCurve accel_curve_;
    SafeControl safe_control_;

    void packResult2Cmd(const catchrobo_msgs::MyRosCmd &target, ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = target.position;
        cmd.v_des = target.velocity;

        //        double accel = (target.velocity - state.velocity) / dt_;
        //        if (accel > target.acceleration_limit)
        //        {
        //            accel = target.acceleration_limit;
        //            cmd.v_des = state.velocity + accel * dt_;
        //            //            ROS_INFO_STREAM("cmd.v_des" << cmd.v_des);
        //        }
        //        if (accel < -target.acceleration_limit)
        //        {
        //            accel = -target.acceleration_limit;
        //            cmd.v_des = state.velocity + accel * dt_;
        //        }

        //        ROS_INFO_STREAM(target);
        // ROS_INFO_STREAM("cmd.v_des" << cmd.v_des);
        cmd.torque_feed_forward = target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }
};
