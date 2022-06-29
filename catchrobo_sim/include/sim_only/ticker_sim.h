#pragma once
#include "motor_driver_bridge/motor_driver_struct.h"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>

class Ticker
{
public:
    Ticker() : nh_(""){};

    void attach(void (*callback_function)(), double dt)
    {
        callback_function_ = callback_function;
        timer_ = nh_.createTimer(ros::Duration(dt), &Ticker::callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    void (*callback_function_)();

    void callback(const ros::TimerEvent &event)
    {
        (*callback_function_)();
    }
};