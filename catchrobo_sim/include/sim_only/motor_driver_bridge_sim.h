#pragma once
#include "motor_driver_bridge/motor_driver_struct.h"

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

    void init(void (*callback_function)(const StateStruct &input), const int (&direction)[4])
    {
        for (size_t i = 0; i < 4; i++)
        {
            direction_[i] = direction[i];
        }
        pub_ = nh_->advertise<catchrobo_msgs::ControlStruct>("motor_driver_cmd", 5);
        pub_enable_ = nh_->advertise<std_msgs::Int8>("motor_driver_enable", 5);
        pub_disable_ = nh_->advertise<std_msgs::Int8>("motor_driver_disable", 5);
        sub_ = nh_->subscribe("motor_driver_state", 50, &MotorDriverBridge::callback, this);

        callback_function_ = callback_function;
    }
    void publish(const ControlStruct &control)
    {
        catchrobo_msgs::ControlStruct data;
        data.id = control.id;
        int direction = direction_[data.id];

        data.p_des = control.p_des * direction;
        data.v_des = control.v_des * direction;
        data.torque_feed_forward = control.torque_feed_forward * direction;
        data.kp = control.kp;
        data.kd = control.kd;
        pub_.publish(data);
    }

    void enableMotor(int id, bool is_enable)
    {
        //// どれか一つでも励起していたら全部enable. 配列にするのが面倒だった
        std_msgs::Int8 msg;
        msg.data = id;
        if (is_enable)
        {
            pub_enable_.publish(msg);
        }
        else
        {
            pub_disable_.publish(msg);
        }
    }
    // void disableMotor(int id)
    // {
    //     std_msgs::Int8 msg;
    //     msg.data = id;
    //     pub_disable_.publish(msg);
    // }
    ////[TODO] 特にsimulatorでやる必要性を感じないので後回し
    void setOrigin(int id) {}

private:
    ros::NodeHandle *nh_;
    ros::Publisher pub_;
    ros::Publisher pub_enable_;
    ros::Publisher pub_disable_;
    ros::Subscriber sub_;
    void (*callback_function_)(const StateStruct &input);

    int direction_[4]; // motorの動く向き. 配列数が直打ちなのが微妙すぎる。

    void callback(const catchrobo_msgs::StateStruct::ConstPtr &input)
    {
        StateStruct data;
        data.id = input->id;

        int direction = direction_[data.id];
        data.position = input->position * direction;
        data.velocity = input->velocity * direction;
        data.torque = input->torque * direction;
        (*callback_function_)(data);
    }
};