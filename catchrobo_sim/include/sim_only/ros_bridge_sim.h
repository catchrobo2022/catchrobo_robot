#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>

// template <class T>
class RosBridge
{
public:
    RosBridge(){};
    void setNodeHandlePtr(ros::NodeHandle *nh)
    {
        nh_ = nh;
    }

    void init(int ros_baudrate, void (*callback_function)(const catchrobo_msgs::MyRosCmd &command), void (*enable_callback_function)(const catchrobo_msgs::EnableCmd &input))
    {
        callback_function_ = callback_function;
        enable_callback_function_ = enable_callback_function;
        pub2ros_ = nh_->advertise<sensor_msgs::JointState>("joint_state_rad", 1);
        pub_finished_flag_ = nh_->advertise<std_msgs::Int8>("finished_flag_topic", 5);
        pub_error_ = nh_->advertise<catchrobo_msgs::ErrorCode>("error", 5);
        sub_from_ros_ = nh_->subscribe("my_joint_control", 50, &RosBridge::rosCallback, this);
        sub_ros_cmd_ = nh_->subscribe("ros_cmd", 50, &RosBridge::rosCallback2, this);
        sub_enable_ = nh_->subscribe("enable_cmd", 1, &RosBridge::enableCmdCallback, this);
    };
    void publishJointState(const sensor_msgs::JointState &joint_state)
    {
        pub2ros_.publish(joint_state);
    };

    void publishFinishFlag(int data)
    {
        std_msgs::Int8 msg;
        msg.data = data;
        pub_finished_flag_.publish(msg);
    };
    void publishError(catchrobo_msgs::ErrorCode msg)
    {
        pub_error_.publish(msg);
    }

    void spin()
    {
        ros::spin();
    };

private:
    ros::NodeHandle *nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub_finished_flag_;
    ros::Publisher pub_error_;
    ros::Subscriber sub_from_ros_;
    ros::Subscriber sub_ros_cmd_;
    ros::Subscriber sub_enable_;
    sensor_msgs::JointState joint_state_;

    void (*callback_function_)(const catchrobo_msgs::MyRosCmd &command);
    void (*enable_callback_function_)(const catchrobo_msgs::EnableCmd &command);

    void rosCallback(const catchrobo_msgs::MyRosCmdArray::ConstPtr &input)
    {
        for (const catchrobo_msgs::MyRosCmd &command : input->command_array)
        {
            (*callback_function_)(command);
        }
    };
    void rosCallback2(const catchrobo_msgs::MyRosCmd::ConstPtr &input)
    {
        (*callback_function_)(*input);
    };
    void enableCmdCallback(const catchrobo_msgs::EnableCmd::ConstPtr &input)
    {
        (*enable_callback_function_)(*input);
    }
};