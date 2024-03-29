#pragma once

#include "catchrobo_sim/controller_interface.h"
#include "catchrobo_sim/accel_curve.h"
#include "catchrobo_sim/safe_control.h"

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

//#

class VelocityControl : public ControllerInterface
{
public:
    VelocityControl(){};
    void init(double dt, SafeControl &safe_control)
    {
        safe_control_ = safe_control;
    }
    virtual void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {
        target_ = cmd;

        // accel_curve_.reset(cmd.jerk_limit, cmd.acceleration_limit, joint_state.velocity, cmd.velocity);
    };

    // dt間隔で呼ばれる想定
    virtual void getCmd(const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, bool &finished)
    {
        packResult2Cmd(target_, command);
        safe_control_.getSafeCmd(state, target_, except_command, command);
        finished = false;
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

        cmd.torque_feed_forward = target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }
};
