#pragma once

#include "catchrobo_sim/position_control.h"
#include "catchrobo_sim/velocity_control.h"
#include "catchrobo_sim/safe_control.h"

#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

class MotorManager
{
public:
    MotorManager() : controller_interface_(&position_control_)
    {
        old_command_.p_des = 0;
        old_command_.v_des = 0;
        old_command_.i_ff = 0;
        old_command_.kp = 0;
        old_command_.kd = 0;

        current_state_.position = 0;
        current_state_.velocity = 0;
        current_state_.current = 0;
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
            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }

        controller_interface_->setRosCmd(cmd, current_state_);
    };

    // dt間隔で呼ばれる
    void getCmd(catchrobo_msgs::ControlStruct &command)
    {
        controller_interface_->getCmd(current_state_, old_command_, command);
        old_command_ = command;
    };

    //高Hz (500Hz)で呼ばれる
    void setCurrentState(const catchrobo_msgs::StateStruct &state)
    {
        current_state_ = state;
    }

    //低Hz (50Hz)で呼ばれる
    void getState(catchrobo_msgs::StateStruct &state)
    {
        state = current_state_;
    }

private:
    ControllerInterface *controller_interface_;
    catchrobo_msgs::StateStruct current_state_;
    catchrobo_msgs::ControlStruct old_command_;

    PositionControl position_control_;
    VelocityControl velocity_control_;
    SafeControl safe_control_;
};
