#pragma once

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>

class RosBridge
{
public:
    RosBridge() : pub2ros_("joint_state_rad", new sensor_msgs::JointState),
                  pub_finished_flag_("finished_flag_topic", new std_msgs::Int8),
                  pub_error_("error", new catchrobo_msgs::ErrorCode),
                  sub_from_ros_("my_joint_control", &RosBridge::rosCallback, this),
                  sub_enable_("enable_motor", &RosBridge::enableCallback, this){};

    void init(int ros_baudrate, void (*callback_function)(const catchrobo_msgs::MyRosCmd &command), void (*enable_callback_function)(const std_msgs::Bool &input))
    {
        callback_function_ = callback_function;
        enable_callback_function_ = enable_callback_function;
        nh_.getHardware()->setBaud(ros_baudrate);
        nh_.initNode();
        nh_.advertise(pub2ros_);
        nh_.advertise(pub_finished_flag_);
        nh_.advertise(pub_error_);
        nh_.subscribe(sub_from_ros_);
        nh_.subscribe(sub_enable_);
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
    void publishError(catchrobo_msgs::ErrorCode msg)
    {
        pub_error_.publish(msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub_finished_flag_;
    ros::Publisher pub_error_;

    ros::Subscriber<catchrobo_msgs::MyRosCmdArray, RosBridge> sub_from_ros_;
    ros::Subscriber<std_msgs::Bool, RosBridge> sub_enable_;

    void (*callback_function_)(const catchrobo_msgs::MyRosCmd &command);
    void (*enable_callback_function_)(const std_msgs::Bool &input);

    void rosCallback(const catchrobo_msgs::MyRosCmdArray &input)
    {

        for (int i = 0; i < input.command_array_length; i++)
        {
            (*callback_function_)(input.command_array[i]);
        }
    };

    void enableCallback(const catchrobo_msgs::EnableCmd &input)
    {
        (*enable_callback_function_)(input);
    }
};
