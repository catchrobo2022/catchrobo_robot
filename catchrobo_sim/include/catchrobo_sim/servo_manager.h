#pragma once

#include "catchrobo_sim/motor_manager.h"
#include "motor_driver_bridge/motor_driver_struct.h"

class ServoManager : public MotorManager
{
public:
    // using MotorManager::MotorManager; //C++11の機能

    ServoManager(){};

    // dt間隔で呼ばれる
    virtual void getCmd(ControlStruct &command, bool &finished)
    {
        MotorManager::getCmd(command, finished);
        StateStruct current_state;
        current_state.position = command.p_des;
        current_state.velocity = command.v_des;
        current_state.torque = command.torque_feed_forward;
        MotorManager::setCurrentState(current_state);
    };
};
