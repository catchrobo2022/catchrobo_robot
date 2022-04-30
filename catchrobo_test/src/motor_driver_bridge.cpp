#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class MotorDriverBridgeInterface
{
    
}

class MotorDriverBridgeSim
{
  public:
    MotorDriverBridgeSim();
    ~MotorDriverBridgeSim(){};

  private:
    void sampleCallback(const sensor_msgs::PointCloud2::ConstPtr &input);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
}

MotorDriverBridgeSim::MotorDriverBridgeSim() : nh_(""), private_nh_("~")
{
    std::string input_topic_name, output_topic_name;
    private_nh_.param("input_topic", input_topic_name, std::string("input"));
    subcriber_ = nh_.subscribe(input_topic_name, 1, &MotorDriverBridgeSim::sampleCallback, this);
    private_nh_.param("output_topic", output_topic_name, std::string("output"));
    publisher_ = nh_.advertise<sensor_msgs::PointCloud>(output_topic_name, 1);
}

void MotorDriverBridgeSim::sampleCallback(const sensor_msgs::PointCloud2::ConstPtr &input){
    if (publisher_.getNumSubscribers() < 1)
        return;
    sensor_msgs::PointCloud2 output;
    output.header = input.header;
    /* 何かの処理 */
    publisher_.publish(output);
}

int main(int argc, char **argv)
{
    MotorDriverBridgeSim sample_node;
    ros::spin();
    return 0;
}