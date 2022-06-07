#pragma once

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

// template <class T>
class RosBridge
{
public:
    RosBridge(){};
    void setNodeHandlePtr(ros::NodeHandle *nh)
    {
        nh_ = nh;
    }

    void init(int ros_baudrate, void (*callback_function)(const catchrobo_msgs::MyRosCmd &command))
    {
        callback_function_ = callback_function;

        pub2ros_ = nh_->advertise<sensor_msgs::JointState>("joint_state_rad", 1);
        pub_finished_flag_ = nh_->advertise<std_msgs::Int8>("finished_flag_topic", 5);
        sub_from_ros_ = nh_->subscribe("my_joint_control", 50, &RosBridge::rosCallback, this);
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

    void spin()
    {
        ros::spin();
    };

private:
    ros::NodeHandle *nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub_finished_flag_;
    ros::Subscriber sub_from_ros_;
    sensor_msgs::JointState joint_state_;
    void (*callback_function_)(const catchrobo_msgs::MyRosCmd &command);

    void rosCallback(const catchrobo_msgs::MyRosCmdArray::ConstPtr &input)
    {
        for (const catchrobo_msgs::MyRosCmd &command : input->command_array)
        {
            (*callback_function_)(command);
        }
    };
};