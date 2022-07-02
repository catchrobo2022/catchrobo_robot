#pragma once

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>
#include <catchrobo_msgs/PegInHoleCmd.h>

class RosBridge
{
public:
    RosBridge() : pub2ros_("joint_rad", new std_msgs::Float32MultiArray),
                  //                  pub_finished_flag_("finished_flag_topic", new std_msgs::Int8),
                  pub_error_("error", new catchrobo_msgs::ErrorCode),
                  //                  sub_from_ros_("my_joint_control", &RosBridge::rosCallback, this),
                  sub_ros_cmd_("ros_cmd", &RosBridge::rosCallback2, this),
                  sub_enable_("enable_cmd", &RosBridge::enableCallback, this),
                  //                  sub_peg_in_hole_("peg_in_hole_cmd", &RosBridge::pegInHoleCallback, this),
                  led_(LED1){};

    void init(int ros_baudrate, void (*callback_function)(const catchrobo_msgs::MyRosCmd &command),
              void (*enable_callback_function)(const catchrobo_msgs::EnableCmd &input),
              void (*peg_in_hole_callack_function)(const std_msgs::Bool &command))

    {
        callback_function_ = callback_function;
        enable_callback_function_ = enable_callback_function;
        nh_.getHardware()->setBaud(ros_baudrate);
        wait(0.5);
        nh_.initNode();
        wait(0.5);
        nh_.advertise(pub2ros_);
        //        wait(0.5);
        //        nh_.advertise(pub_finished_flag_);
        wait(0.5);
        nh_.advertise(pub_error_);
        //        nh_.subscribe(sub_from_ros_);
        wait(0.5);
        nh_.subscribe(sub_enable_);
        wait(0.5);
        nh_.subscribe(sub_ros_cmd_);
        //        wait(0.5);
        //        nh_.subscribe(sub_peg_in_hole_);
    };
    void publishJointState(const std_msgs::Float32MultiArray &joint_state)
    {
        pub2ros_.publish(&joint_state);
    };

    void publishFinishFlag(int data)
    {
        std_msgs::Int8 msg;
        msg.data = data;
        //        pub_finished_flag_.publish(&msg);
    };

    void spin()
    {
        while (1)
        {
            nh_.spinOnce();
            led_ = !led_;
            wait_ms(1000);
        }
    };
    void publishError(const catchrobo_msgs::ErrorCode &msg)
    {
        pub_error_.publish(&msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub2ros_;
    //    ros::Publisher pub_finished_flag_;
    ros::Publisher pub_error_;

    //    ros::Subscriber<catchrobo_msgs::MyRosCmdArray, RosBridge> sub_from_ros_;
    ros::Subscriber<catchrobo_msgs::MyRosCmd, RosBridge> sub_ros_cmd_;
    ros::Subscriber<catchrobo_msgs::EnableCmd, RosBridge> sub_enable_;
    //    ros::Subscriber<std_msgs::Bool, RosBridge> sub_peg_in_hole_;

    DigitalOut led_;

    void (*callback_function_)(const catchrobo_msgs::MyRosCmd &command);
    void (*enable_callback_function_)(const catchrobo_msgs::EnableCmd &input);
    void (*peg_in_hole_callack_function_)(const std_msgs::Bool &command);

    void rosCallback(const catchrobo_msgs::MyRosCmdArray &input)
    {
        for (int i = 0; i < input.command_array_length; i++)
        {
            (*callback_function_)(input.command_array[i]);
        }
    };
    void rosCallback2(const catchrobo_msgs::MyRosCmd &input)
    {
        (*callback_function_)(input);
    };
    void enableCallback(const catchrobo_msgs::EnableCmd &input)
    {
        (*enable_callback_function_)(input);
    }
    void pegInHoleCallback(const std_msgs::Bool &input)
    {
        (*peg_in_hole_callack_function_)(input);
    }
};
