#pragma once
#include "catchrobo_sim/motor_driver_struct.h"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>

class TickerBridge
{
public:
    TickerBridge() : nh_(""){};

    void init(double dt, void (*callback_function)())
    {
        callback_function_ = callback_function;
        timer_ = nh_.createTimer(ros::Duration(dt), &TickerBridge::callback, this);
    };
    void callback(const ros::TimerEvent &event)
    {
        (*callback_function_)();
    };

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    void (*callback_function_)();
};