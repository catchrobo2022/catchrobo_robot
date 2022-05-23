#pragma once

#include "catchrobo_sim/position_control.h"
#include "catchrobo_sim/velocity_control.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/motor_driver_struct.h"

#include <catchrobo_msgs/MyRosCmd.h>

class MotorManager
{
public:
    MotorManager() : controller_interface_(&position_control_)
    {
        old_command_.p_des = 0;
        old_command_.v_des = 0;
        old_command_.torque_feed_forward = 0;
        old_command_.kp = 0;
        old_command_.kd = 0;

        current_state_.position = 0;
        current_state_.velocity = 0;
        current_state_.torque = 0;
    };

    void init(double dt)
    {
        double cbf_params = 1;
        safe_control_.setCBFparams(cbf_params);
        position_control_.init(dt, safe_control_);
        velocity_control_.init(dt, safe_control_);
    }

    //低Hz (1 Hzとか)で呼ばれる
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    {
        switch (cmd.mode)
        {
        case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:
            controller_interface_ = &position_control_;
            break;
        case catchrobo_msgs::MyRosCmd::VELOCITY_CTRL_MODE:
            controller_interface_ = &velocity_control_;
            break;

        default:
            //            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }

        controller_interface_->setRosCmd(cmd, current_state_);
    };

    // dt間隔で呼ばれる. servo classではoverrideされる。
    virtual void getCmd(ControlStruct &command, bool &finished)
    {
        controller_interface_->getCmd(current_state_, old_command_, command, finished);
        old_command_ = command;
    };

    //高Hz (500Hz)で呼ばれる
    void setCurrentState(const StateStruct &state)
    {
        current_state_ = state;
    }

    //低Hz (50Hz)で呼ばれる
    void getState(StateStruct &state)
    {
        state = current_state_;
    }

private:
    ControllerInterface *controller_interface_;
    StateStruct current_state_;
    ControlStruct old_command_;

    PositionControl position_control_;
    VelocityControl velocity_control_;
    SafeControl safe_control_;
};
