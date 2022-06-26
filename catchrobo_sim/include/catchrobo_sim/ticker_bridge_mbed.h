#pragma once
#include "catchrobo_sim/motor_driver_struct.h"
template <class T>
class TickerBridge
{
public:
    TickerBridge() : nh_(""){};

    void init(double dt, void (T::*callback_function)(), T *obj)
    {
        obj_ = obj;
        callback_function_ = callback_function;
        timer_.attach(callback(this, &TickerBridge::timer_callback), dt);
    };

private:
    ros::NodeHandle nh_;
    Ticker timer_;
    void (T::*callback_function_)();
    T *obj_;

    void timer_callback()
    {
        (obj_->*callback_function_)();
    };
};