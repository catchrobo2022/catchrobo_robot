#pragma once
#include "catchrobo_sim/motor_driver_struct.h"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>

class MotorDriverBridge
{
public:
    MotorDriverBridge(){};

    void setNodeHandlePtr(ros::NodeHandle *nh)
    {
        nh_ = nh;
    }

    void init(void (*callback_function)(const StateStruct &input))
    {
        pub_ = nh_->advertise<catchrobo_msgs::ControlStruct>("motor_driver_cmd", 1);
        sub_ = nh_->subscribe("motor_driver_state", 50, &MotorDriverBridge::callback, this);

        callback_function_ = callback_function;
    }
    void publish(const ControlStruct &control)
    {
        catchrobo_msgs::ControlStruct data;
        data.id = control.id;
        data.p_des = control.p_des;
        data.v_des = control.v_des;
        data.torque_feed_forward = control.torque_feed_forward;
        data.kp = control.kp;
        data.kd = control.kd;
        pub_.publish(data);
    }
    void enable_motor(int id) {}

private:
    ros::NodeHandle *nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    void (*callback_function_)(const StateStruct &input);
    void callback(const catchrobo_msgs::StateStruct::ConstPtr &input)
    {
        StateStruct data;
        data.id = input->id;
        data.position = input->position;
        data.velocity = input->velocity;
        data.torque = input->torque;
        (*callback_function_)(data);
    }
};