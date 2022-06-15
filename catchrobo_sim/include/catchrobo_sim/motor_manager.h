#pragma once

#include "catchrobo_sim/position_control.h"
#include "catchrobo_sim/direct_control.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/go_origin_control.h"
#include "motor_driver_bridge/motor_driver_struct.h"

#include <catchrobo_msgs/MyRosCmd.h>

class MotorManager
{
public:
    MotorManager()
    {
        old_command_.p_des = 0;
        old_command_.v_des = 0;
        old_command_.torque_feed_forward = 0;
        old_command_.kp = 0;
        old_command_.kd = 0;

        current_state_.position = 0;
        current_state_.velocity = 0;
        current_state_.torque = 0;
        double cbf_params = 1;
        safe_control_.setCBFparams(cbf_params);
    };

    //低Hz (1 Hzとか)で呼ばれる
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    {
        switch (cmd.mode)
        {
        case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:
            position_control_.setRosCmd(cmd, current_state_);
            break;
        case catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE:
            direct_control_.setRosCmd(cmd, current_state_);
            break;
        case catchrobo_msgs::MyRosCmd::GO_ORIGIN_MODE:
            go_origin_control_.setRosCmd(cmd, current_state_);
            break;

        default:
            //            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }
        ros_cmd_ = cmd;
    };

    // dt間隔で呼ばれる. servo classではoverrideされる。
    virtual void getCmd(ControlStruct &command, ControlResult::ControlResult &result)
    {
        switch (ros_cmd_.mode)
        {
        case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:

            position_control_.getCmd(current_state_, old_command_, command, result);
            safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            break;
        case catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE:
            direct_control_.getCmd(current_state_, old_command_, command, result);
            safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            break;

        case catchrobo_msgs::MyRosCmd::GO_ORIGIN_MODE:
            go_origin_control_.getCmd(current_state_, old_command_, command, result);
            break;

        default:
            command = old_command_;
            //            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }
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

    void getRosCmd(catchrobo_msgs::MyRosCmd &ros_cmd)
    {
        ros_cmd = ros_cmd_;
    }
    void nextStep(float dt)
    {
        position_control_.nextStep(dt);
    };
    bool IsPegInHoleMode()
    {
        return (ros_cmd_.mode == catchrobo_msgs::MyRosCmd::PEG_IN_HOLE_MODE);
    }

private:
    StateStruct current_state_;
    ControlStruct old_command_;

    catchrobo_msgs::MyRosCmd ros_cmd_;

    PositionControl position_control_;
    DirectControl direct_control_;
    SafeControl safe_control_;
    GoOriginControl go_origin_control_;
};
