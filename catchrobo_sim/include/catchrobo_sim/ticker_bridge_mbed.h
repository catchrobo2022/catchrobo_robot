#pragma once
#include "catchrobo_sim/motor_driver_struct.h"

class TickerBridge
{
public:
    TickerBridge(){};

    void init(double dt, void (*callback_function)())
    {
        callback_function_ = callback_function;
        timer_.attach(callback(this, &TickerBridge::timer_callback), dt);
    };

private:
    ros::NodeHandle nh_;
    Ticker timer_;
    void (*callback_function_)();

    void timer_callback()
    {
        (*callback_function_)();
    };
};