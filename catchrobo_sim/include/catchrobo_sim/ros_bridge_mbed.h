#pragma once

#include <ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

template <class T>
class RosBridge
{
public:
    RosBridge() : pub2ros_("my_joint_state", new sensor_msgs::JointState),
                  pub_finished_flag_("finished_flag_topic", new std_msgs::Int8),
                  sub_from_ros_("my_joint_control", &RosBridge::rosCallback, this){};

    void init(int ros_baudrate, void (T::*callback_function)(const catchrobo_msgs::MyRosCmdArray &input), T *obj)
    {
        nh_.getHardware()->setBaud(ros_baudrate);
        nh_.initNode();
        nh_.advertise(pub2ros_);
        nh_.advertise(pub_finished_flag_);
        nh_.subscribe(sub_from_ros_);

        obj_ = obj;
        callback_function_ = callback_function;
    };
    void publishJointState(const sensor_msgs::JointState &joint_state)
    {
        pub2ros_.publish(&joint_state);
    };

    void publishFinishFlag(const std_msgs::Int8 &flag)
    {
        pub_finished_flag_.publish(&flag);
    };
    void rosCallback(const catchrobo_msgs::MyRosCmdArray &input)
    {
        callback_function_(input);
    };

private:
    ros::NodeHandle nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub_finished_flag_;
    ros::Subscriber<catchrobo_msgs::MyRosCmdArray, RosBridge> sub_from_ros_;
    void (T::*callback_function_)(const catchrobo_msgs::MyRosCmdArray &input);
    T *obj_;
};