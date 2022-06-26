#pragma once

#include <ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

template <class T>
class RosBridge
{
public:
    RosBridge() : pub2ros_("joint_state_radius", new sensor_msgs::JointState),
                  pub_finished_flag_("finished_flag_topic", new std_msgs::Int8),
                  sub_from_ros_("my_joint_control", &RosBridge::rosCallback, this){};

    void init(int ros_baudrate, void (T::*callback_function)(const catchrobo_msgs::MyRosCmd &command), T *obj)
    {

        obj_ = obj;
        callback_function_ = callback_function;
        nh_.getHardware()->setBaud(ros_baudrate);
        nh_.initNode();
        nh_.advertise(pub2ros_);
        nh_.advertise(pub_finished_flag_);
        nh_.subscribe(sub_from_ros_);
    };
    void publishJointState(const sensor_msgs::JointState &joint_state)
    {
        pub2ros_.publish(&joint_state);
    };

    void publishFinishFlag(int data)
    {
        std_msgs::Int8 msg;
        msg.data = data;
        pub_finished_flag_.publish(&msg);
    };

    void spin()
    {
        while (1)
        {
            nh_.spinOnce();
            wait_ms(1000);
        }
    };

private:
    ros::NodeHandle nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub_finished_flag_;
    ros::Subscriber<catchrobo_msgs::MyRosCmdArray, RosBridge> sub_from_ros_;
    sensor_msgs::JointState joint_state_;
    void (T::*callback_function_)(const catchrobo_msgs::MyRosCmd &command);
    T *obj_;

    void rosCallback(const catchrobo_msgs::MyRosCmdArray &input)
    {

        for (int i = 0; i < input.command_array_length; i++)
        {
            (obj_->*callback_function_)(input.command_array[i]);
        }
    };
};
