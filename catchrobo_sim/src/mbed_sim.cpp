#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/LinearRobotControl.h>

#include <string>
#include <vector>

class Controller
{
public:

    Controller(){
        for (size_t i = 0; i < 3; i++)
        {
            cmd_[i].id = i;
        }
    };
    void setTarget(const catchrobo_msgs::LinearRobotControl &target){
        for (size_t i = 0; i < 3; i++)
        {
            cmd_[i].p_des = target.position[i];
        }
        
    };
    void getCmd(int id, catchrobo_msgs::ControlStruct &cmd){
        cmd = cmd_[id];
    };
private:
    catchrobo_msgs::ControlStruct cmd_[3];
};

class MbedSim
{
public:
    MbedSim(): nh_(""), private_nh_("~")
    {

        joint_state_.name = std::vector<std::string>{"arm/joint1", "arm/joint2", "arm/joint3"};

        int size = joint_state_.name.size();
        joint_state_.position = std::vector<double>(size, 0);
        joint_state_.velocity = std::vector<double>(size, 0);
        joint_state_.effort = std::vector<double>(size, 0);

        RosSetup();
    };

private:
    Controller controller_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pub2ros_;
    ros::Publisher pub2motor_;
    ros::Subscriber sub_from_motor_;
    ros::Subscriber sub_from_ros_;
    ros::Timer mbed2ros_timer_;
    ros::Timer mbed2motor_timer_;

    float motor_driver_cmd_dt_;
    sensor_msgs::JointState joint_state_;

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
        sub_from_motor_ = nh_.subscribe(input_topic_name, 10, &MbedSim::onMsgReceived, this);

        private_nh_.param<std::string>("input_topic_from_ros", input_topic_name, "my_joint_control");
        sub_from_ros_ = nh_.subscribe(input_topic_name, 10, &MbedSim::rosCallback, this);
    };

    void mbed2RosTimerCallback(const ros::TimerEvent &event)
    {
        pub2ros_.publish(joint_state_);
    };

    void mbed2MotorDriverTimerCallback(const ros::TimerEvent &event)
    {   
        catchrobo_msgs::ControlStruct cmd;
        for (size_t i = 0; i < 3; i++)
        {
            controller_.getCmd(i, cmd);
            pack_cmd(cmd);
        }        
    };

    void rosCallback(const catchrobo_msgs::LinearRobotControl::ConstPtr &input){
        controller_.setTarget(*input);
    };

    void onMsgReceived(const catchrobo_msgs::StateStruct::ConstPtr &input)
    {
        int id = input->id;
        joint_state_.position[id] = input->position;
        joint_state_.velocity[id] = input->velocity;
        joint_state_.effort[id] = input->current;
    };

    void pack_cmd(const catchrobo_msgs::ControlStruct &cmd){
        pub2motor_.publish(cmd);
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mbed_sim");
    MbedSim sample_node;
    ros::spin();
    return 0;
}