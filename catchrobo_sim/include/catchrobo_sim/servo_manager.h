#pragma once

#include "catchrobo_sim/motor_manager.h"

#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <ros/ros.h>

class ServoManager : public MotorManager
{
public:
    using MotorManager::MotorManager;

    // ServoManager()
    // {
    //     ROS_INFO_STREAM("servo start");
    // };

    // dt間隔で呼ばれる
    void getCmd(catchrobo_msgs::ControlStruct &command, bool &finished) override
    {
        MotorManager::getCmd(command, finished);
        catchrobo_msgs::StateStruct current_state;
        current_state.position = command.p_des;
        current_state.velocity = command.v_des;
        current_state.current = command.i_ff;
        MotorManager::setCurrentState(current_state);

        ROS_INFO_STREAM("servo");
        ROS_INFO_STREAM(current_state);
    };
};
