#include "catchrobo_sim/robot_manager.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

#include <string>
#include <vector>


class MbedSim
{
public:
    MbedSim(): nh_(""), private_nh_("~")
    {
        RosSetup();
        robot_manager_.init(motor_driver_cmd_dt_);
        ROS_INFO("init finish");
    };

private:
    RobotManager robot_manager_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub2motor_;
    ros::Subscriber sub_from_motor_;
    ros::Subscriber sub_from_ros_;
    ros::Timer mbed2ros_timer_;
    ros::Timer mbed2motor_timer_;

    float motor_driver_cmd_dt_;

    void RosSetup()
    {
        float mbed2ros_dt;
        private_nh_.param<float>("mbed2ros_dt", mbed2ros_dt, 0.02);
        mbed2ros_timer_ = nh_.createTimer(ros::Duration(mbed2ros_dt), &MbedSim::mbed2RosTimerCallback, this);

        private_nh_.param<float>("motor_driver_cmd_dt_", motor_driver_cmd_dt_, 0.002);
        mbed2motor_timer_ = nh_.createTimer(ros::Duration(motor_driver_cmd_dt_), &MbedSim::mbed2MotorDriverTimerCallback, this);

        std::string input_topic_name, output_topic_name;
        private_nh_.param<std::string>("output_topic_to_ros", output_topic_name, "my_joint_state");
        pub2ros_ = nh_.advertise<sensor_msgs::JointState>(output_topic_name, 1);

        private_nh_.param<std::string>("output_topic_to_motor", output_topic_name, "motor_driver_cmd");
        pub2motor_ = nh_.advertise<catchrobo_msgs::ControlStruct>(output_topic_name, 1);

        private_nh_.param<std::string>("input_topic", input_topic_name, "motor_driver_state");
        sub_from_motor_ = nh_.subscribe(input_topic_name, 50, &MbedSim::CANCallback, this);

        private_nh_.param<std::string>("input_topic_from_ros", input_topic_name, "my_joint_control");
        sub_from_ros_ = nh_.subscribe(input_topic_name, 50, &MbedSim::rosCallback, this);
    };

    void mbed2RosTimerCallback(const ros::TimerEvent &event)
    {

        sensor_msgs::JointState joint_state;
        robot_manager_.getJointState(joint_state);
        pub2ros_.publish(joint_state);
    };

    void mbed2MotorDriverTimerCallback(const ros::TimerEvent &event)
    {   
        catchrobo_msgs::ControlStruct cmd;
        for (size_t i = 0; i < 3; i++)
        {
            robot_manager_.getCmd(i, cmd);
            pub2motor_.publish(cmd);
        }        
    };

    void rosCallback(const catchrobo_msgs::MyRosCmdArray::ConstPtr &input){
        robot_manager_.setRosCmd(*input);
    };

    void CANCallback(const catchrobo_msgs::StateStruct::ConstPtr &input)
    {
        robot_manager_.setCurrentState(*input);
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mbed_sim");
    ROS_INFO("START");
    MbedSim sample_node;
    ros::spin();
    return 0;
}