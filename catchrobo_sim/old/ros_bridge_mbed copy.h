#pragma once

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

class RosBridge
{
public:
    RosBridge() : pub2ros_("my_joint_state", new sensor_msgs::JointState),
                  pub_finished_flag_("finished_flag_topic", new std_msgs::Int8),
                  sub_from_ros_("my_joint_control", &RosBridge::rosCallback, this),
                  sub_zero_position_("set_zero_position", &RosBridge::zeroPositionCallback, this){};

    void init(int ros_baudrate, void (*callback_function)(const catchrobo_msgs::MyRosCmd &command), void (*zero_position_callback)())
    {
        callback_function_ = callback_function;
        zero_position_callback_ = zero_position_callback;
        nh_.getHardware()->setBaud(ros_baudrate);
        nh_.initNode();
        nh_.advertise(pub2ros_);
        nh_.advertise(pub_finished_flag_);
        nh_.subscribe(sub_from_ros_);
        nh_.subscribe(sub_zero_position_);
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
    ros::Subscriber<std_msgs::Empty, RosBridge> sub_zero_position_;
    sensor_msgs::JointState joint_state_;
    void (*callback_function_)(const catchrobo_msgs::MyRosCmd &command);
    void (*zero_position_callback_)();

    void rosCallback(const catchrobo_msgs::MyRosCmdArray &input)
    {

        for (int i = 0; i < input.command_array_length; i++)
        {
            (*callback_function_)(input.command_array[i]);
        }
    };

    void zeroPositionCallback(const std_msgs::Empty &input)
    {
        zero_position_callback_();
    }
};