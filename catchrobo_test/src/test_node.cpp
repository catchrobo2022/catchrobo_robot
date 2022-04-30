#include <ros/ros.h>
#include <catchrobo_msgs/LinearRobotControl.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

// #include <catchrobo_msgs/MotorDriver.h>

// mbed simulator. ROS周りの実装. mbedでは書き方が変わるであろうROS周りをまとめたもの。
class MbedSimRos
{
public:
    MbedSimRos();
    ~MbedSimRos(){};
    void spin();

private:
    void cmdCallback(const catchrobo_msgs::LinearRobotControl::ConstPtr &input);
    void timer_callback(const ros::TimerEvent &e);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher joint_state_pub_;
    
    ros::Subscriber subscriber_;

    sensor_msgs::JointState state_;
    catchrobo_msgs::LinearRobotControl cmd_;
    ros::Timer timer_;
    

    // //motor driverへの指示。mbedではCANに変える。
    // std::vector<ros::Publisher> motor_pubs_;
    // std::vector<ros::Subscriber> motor_subs_;
};

MbedSimRos::MbedSimRos() : nh_(""), private_nh_("~")
{
    std::string input_topic_name, output_topic_name;
    private_nh_.param("input_topic", input_topic_name, std::string("my_joint_control"));
    subscriber_ = nh_.subscribe(input_topic_name, 1, &MbedSimRos::cmdCallback, this);
    private_nh_.param("output_topic", output_topic_name, std::string("my_joint_state"));
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(output_topic_name, 1);
    

    // joint stateのpublishは高周期で行う
    double frequency;
    private_nh_.param("mbed2ros_freq", frequency, 0.1);
    timer_ = nh_.createTimer(ros::Duration(frequency), &MbedSimRos::timer_callback, this);
    
    state_ = sensor_msgs::JointState();
    //// [TODO] joint名を新robotに合わせる
    std::vector<std::string> name = {"arm/joint1", "arm/joint2", "arm/joint3", "gripper/joint1"}; 
    state_.name = name;

    cmd_ = catchrobo_msgs::LinearRobotControl();


    // motor_pubs_ = {
    //      nh_.advertise<catchrobo_msgs::MotorDriver>("joint1/cmd", 1);
    // }

    // motor_subs = {
    //     nh_.subscribe(input_topic_name, 1, &MbedSimRos::cmdCallback, this);
    // }
}



void MbedSimRos::cmdCallback(const catchrobo_msgs::LinearRobotControl::ConstPtr &input)
{   
    cmd_ = *input;
}

void MbedSimRos::timer_callback(const ros::TimerEvent &e)
{   
    // // 一定間隔でROSにjoint state送信
    // if (joint_state_pub_.getNumSubscribers() < 1)
    //     return;
    joint_state_pub_.publish(state_);
}

void MbedSimRos::spin()
{
    //軌道計画
    state_.position = cmd_.position;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "rsj_robot_test_node");
    MbedSimRos sample_node;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sample_node.spin();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}