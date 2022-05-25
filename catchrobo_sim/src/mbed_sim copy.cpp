#include "catchrobo_sim/robot_manager.h"
#include "catchrobo_sim/motor_driver_struct.h"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/MyRosCmdArray.h>

#include <string>
#include <vector>

#include "catchrobo_sim/mbed_manager.h"

class MbedSim
{
public:
    MbedSim() : nh_(""), private_nh_("~")
    {
        RosSetup();
        sensor_msgs::JointState joint_state_init;
        joint_state_init.name = std::vector<std::string>{"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"};

        int joint_num = joint_state_init.name.size();
        joint_state_init.position = std::vector<double>(joint_num, 0);
        joint_state_init.velocity = std::vector<double>(joint_num, 0);
        joint_state_init.effort = std::vector<double>(joint_num, 0);

        robot_manager_.init(motor_driver_cmd_dt_, joint_state_init);
        ROS_INFO("init finish");
    };

private:
    RobotManager robot_manager_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub2motor_;
    ros::Publisher pub_finished_flag_;
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

        std::string finished_flag;
        private_nh_.param<std::string>("finished_flag_topic", finished_flag, "finished_flag_topic");
        pub_finished_flag_ = nh_.advertise<std_msgs::Int8>(finished_flag, 5);

        private_nh_.param<std::string>("output_topic_to_motor", output_topic_name, "motor_driver_cmd");
        pub2motor_ = nh_.advertise<catchrobo_msgs::ControlStruct>(output_topic_name, 10);

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
        for (size_t i = 0; i < 4; i++)
        {
            bool finished = false;
            ControlStruct cmd;
            robot_manager_.getCmd(i, cmd, finished);

            catchrobo_msgs::ControlStruct cmd_msg;
            cmd_msg.id = cmd.id;
            cmd_msg.p_des = cmd.p_des;
            cmd_msg.v_des = cmd.v_des;
            cmd_msg.torque_feed_forward = cmd.torque_feed_forward;
            cmd_msg.kp = cmd.kp;
            cmd_msg.kd = cmd.kd;

            pub2motor_.publish(cmd_msg);
            if (finished)
            {
                std_msgs::Int8 data;
                data.data = i;
                pub_finished_flag_.publish(data);
            }
        }
    };

    void rosCallback(const catchrobo_msgs::MyRosCmdArray::ConstPtr &input)
    {
        for (const catchrobo_msgs::MyRosCmd &command : input->command_array)
        {
            robot_manager_.setRosCmd(command);
        }
    };

    void CANCallback(const catchrobo_msgs::StateStruct::ConstPtr &input)
    {
        StateStruct state;
        state.id = input->id;
        state.position = input->position;
        state.velocity = input->velocity;
        state.torque = input->torque;
        robot_manager_.setCurrentState(state);
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mbed_sim");
    ROS_INFO("START");
    // MbedSim sample_node;
    MbedManager sample_node;
    sample_node.init(0.02, 0.002);
    ros::spin();
    return 0;
}