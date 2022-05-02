#pragma once

#include "catchrobo_sim/position_control.h"

#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>


class MotorManager
{
public:
    MotorManager():controller_interface_(position_control_){
        old_command_.p_des = 0;
        old_command_.v_des = 0;
        old_command_.i_ff = 0;
        old_command_.kp = 0;
        old_command_.kd = 0;
    };

    void init(double dt){
        position_control_.init(dt);
    }

    //低Hz (1 Hzとか)で呼ばれる
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    {   

        // switch (cmd.mode)
        // {
        // case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:
        //     /* code */
        //     break;
        
        // default:
        //     break;
        // }
        controller_interface_ = position_control_;
        controller_interface_.setRosCmd(cmd, current_state_);
    };

    // dt間隔で呼ばれる
    void getCmd(catchrobo_msgs::ControlStruct &cmd)
    {
        controller_interface_.getCmd(old_command_, cmd);



        old_command_ = cmd;
    };

    //高Hz (500Hz)で呼ばれる
    void setCurrentState(const catchrobo_msgs::StateStruct &state){
        current_state_ = state;
    }


    //低Hz (50Hz)で呼ばれる
    void getState(catchrobo_msgs::StateStruct &state){
        state = current_state_;
    }

private:
    ControllerInterface &controller_interface_;
    catchrobo_msgs::StateStruct current_state_;
    catchrobo_msgs::ControlStruct old_command_;

    PositionControl position_control_;
};
