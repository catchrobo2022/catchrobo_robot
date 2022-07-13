#include <ros/ros.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/StateStruct.h>
#include <std_msgs/Int8.h>
#include <string>

class MotorDriverSim
{
public:
    MotorDriverSim();
    // ~MotorDriverSim(){};

private:
    void sampleCallback(const catchrobo_msgs::ControlStruct::ConstPtr &input);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Subscriber sub_enable_;
    ros::Subscriber sub_disable_;

    // void timerCallback(const ros::TimerEvent &event);
    void timerCallback();

    ros::Timer timer_;

    int id_;
    double dt_;

    catchrobo_msgs::ControlStruct cmd_;
    catchrobo_msgs::StateStruct state_;
    catchrobo_msgs::StateStruct old_state_;
    bool is_enable_;

    ros::Time t_;

    void enable(const std_msgs::Int8::ConstPtr &input)
    {
        if (input->data == id_)
        {
            is_enable_ = true;
            ROS_INFO("enable %d", input->data);
        }
    }
    void disable(const std_msgs::Int8::ConstPtr &input)
    {
        if (input->data == id_)
        {
            is_enable_ = false;
        }
    }
};

MotorDriverSim::MotorDriverSim() : nh_(""), private_nh_("~")
{
    is_enable_ = false;
    private_nh_.param<int>("motor_id", id_, -1);
    state_.id = int(id_);

    std::string input_topic_name, output_topic_name;
    private_nh_.param<std::string>("input_topic", input_topic_name, "input_topic");
    subscriber_ = nh_.subscribe(input_topic_name, 5, &MotorDriverSim::sampleCallback, this);

    private_nh_.param<std::string>("output_topic", output_topic_name, "output_topic");
    publisher_ = nh_.advertise<catchrobo_msgs::StateStruct>(output_topic_name, 1);

    private_nh_.param<double>("dt", dt_, 0.1);
    // timer_ = nh_.createTimer(ros::Duration(dt_), &MotorDriverSim::timerCallback, this);
    private_nh_.param<double>("init_position_rad", state_.position, 0);

    t_ = ros::Time::now();
    sub_enable_ = nh_.subscribe("/motor_driver_enable", 5, &MotorDriverSim::enable, this);
    sub_disable_ = nh_.subscribe("/motor_driver_disable", 5, &MotorDriverSim::disable, this);

    ROS_INFO_STREAM("motor id: " << id_ << " dt: " << dt_);
}

void MotorDriverSim::sampleCallback(const catchrobo_msgs::ControlStruct::ConstPtr &input)
{
    if (input->id == id_)
    {
        cmd_ = *input;
        timerCallback();
    }
}

// void MotorDriverSim::timerCallback(const ros::TimerEvent &event)
void MotorDriverSim::timerCallback()
{
    ros::Time now = ros::Time::now();
    ros::Duration ros_dt = now - t_;
    double dt = ros_dt.toSec();
    t_ = now;
    // ROS_INFO_STREAM("dt : " << dt);
    // if (dt > 0.1)
    // {
    //     //時間が立ちすぎていたら一度指示スキップ
    //     return;
    // }
    // double vel = 0;
    // double torque_ref = cmd_.kp * (cmd_.p_des - state_.position) + cmd_.kd * (cmd_.v_des - state_.velocity) + cmd_.torque_feed_forward;
    // if (is_enable_)
    // {
    //     vel = cmd_.kp * (cmd_.p_des - state_.position) + cmd_.kd * cmd_.v_des;
    // }
    // state_.velocity = vel;
    // state_.position += state_.velocity * dt_;

    float current_velocity = state_.velocity;
    float current_position = state_.position;

    if (cmd_.kd > 0)
    {
        state_.velocity = cmd_.v_des;
        state_.position += state_.velocity * dt;
    }
    if (cmd_.kp > 0)
    {
        float weight = 0.5;
        state_.position = weight * cmd_.p_des + (1 - weight) * current_position;
        state_.velocity = (state_.position - current_position) / dt;
    }
    // state_.position += cmd_.kp * (cmd_.p_des - state_.position) + cmd_.kd * cmd_.v_des * dt_;
    // state_.velocity = (state_.position - old_state_.position) / dt_;
    // state_.torque = (state_.velocity - current_velocity) / dt;
    state_.torque = cmd_.torque_feed_forward;

    publisher_.publish(state_);
    old_state_ = state_;
    // if (cmd_.id == 0)
    // {
    //     if (state_.velocity > 200)
    //         ROS_INFO_STREAM("dt: " << dt << " v: " << state_.velocity);
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_driver_sim");
    MotorDriverSim sample_node;
    ros::spin();
    return 0;
}