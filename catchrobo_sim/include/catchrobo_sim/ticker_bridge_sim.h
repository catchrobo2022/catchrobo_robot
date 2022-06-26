#pragma once
#include "catchrobo_sim/motor_driver_struct.h"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>

template <class T>
class TickerBridge
{
public:
    TickerBridge() : nh_(""){};

    void init(double dt, void (T::*callback_function)(), T *obj)
    {
        obj_ = obj;
        callback_function_ = callback_function;
        timer_ = nh_.createTimer(ros::Duration(dt), &TickerBridge::callback, this);
    };
    void callback(const ros::TimerEvent &event)
    {
        (obj_->*callback_function_)();
    };

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    void (T::*callback_function_)();
    T *obj_;
};