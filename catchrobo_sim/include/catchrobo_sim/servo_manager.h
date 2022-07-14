#pragma once

#include "catchrobo_sim/motor_manager.h"
#include "motor_driver_bridge/motor_driver_struct.h"

class ServoManager : public MotorManager
{
    float ONE_ACTION_S_;

public:
    // using MotorManager::MotorManager; //C++11の機能

    ServoManager() : ONE_ACTION_S_(0.2){};

    // dt間隔で呼ばれる
    virtual void getCmd(ControlStruct &command, ControlResult::ControlResult &finished)
    {
        catchrobo_msgs::MyRosCmd ros_cmd;
        getRosCmd(ros_cmd);
        MotorManager::getCmd(command, finished);

        command.p_des = ros_cmd.position;
        StateStruct current_state;
        current_state.position = command.p_des;
        current_state.velocity = command.v_des;
        current_state.torque = command.torque_feed_forward;
        MotorManager::setCurrentState(current_state);
        // finished = ControlResult::RUNNING;
        // if (t_ > ONE_ACTION_S_)
        // {
        //     finished = ControlResult::FINISH;
        // }
    };
};
