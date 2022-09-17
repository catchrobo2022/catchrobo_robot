//   #define USE_MBED

#ifdef USE_MBED
#include "mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#include "catchrobo_sim/ros_bridge_mbed.h"
const float MBED2ROS_DT = 0.1; // 10Hz
#else
#include <ros/ros.h>
#include "sim_only/mbed.h"
#include "sim_only/motor_driver_bridge_sim.h"
#include "sim_only/ros_bridge_sim.h"
#include "sim_only/ticker_sim.h"
const float MBED2ROS_DT = 0.01; // 10Hz
#endif

#include <std_msgs/Float32MultiArray.h>
#include "catchrobo_sim/robot_manager.h"
#include "catchrobo_sim/gripper_manager.h"

// const float SPIN_FREQUENCY_s = 0.001;
const float MBED2GRIPPER_DT = 0.1;
const float MBED2MOTOR_DT = 0.005; // 1000Hz
const int SERIAL_BAUD_RATE = 115200;
const float ARRIVE_THRESHOLD_RAD[] = {0.1, 0.1, 0.1};
const float FRICTION[] = {0.2, 0, 0};
const float GRIPPER_THRESHOLD_RAD = 0.1;
const float ESTIMATE_ERROR_LIMIT_RAD = 0.5;

MotorDriverBridge motor_driver_bridge;
RosBridge ros_bridge;
RobotManager robot_manager;
GripperManager gripper_manager;
EnableManager enable_manager;

void enableAll(bool is_enable)
{
    for (int i = 0; i < N_MOTORS; i++)
    {
        motor_driver_bridge.enableMotor(i, is_enable);
        wait(0.1);
    }
}

void motorDriverCallback(const StateStruct &input)
{
    robot_manager.setCurrentState(input);
};

void rosCallback(const catchrobo_msgs::MyRosCmd &command)
{
    if (command.mode == catchrobo_msgs::MyRosCmd::PEG_IN_HOLE_MODE)
    {
        bool result[N_MOTORS] = {};
        robot_manager.arriveCheck(result);
        for (int i = 0; i < N_MOTORS; i++)
        {
            if (result[i])
            {
                catchrobo_msgs::ErrorCode error;
                error.id = i;
                error.error_code = catchrobo_msgs::ErrorCode::FINISH;
                ros_bridge.publishError(error);
                // ros_bridge.publishFinishFlag(i);
                // ros::Duration dt = ros::Time::now() - t;
                // ROS_INFO_STREAM("dt " << dt.toSec() );
            }
        }
        return;
    }
    robot_manager.setRosCmd(command);
    gripper_manager.setRosCmd(command);
}

void mbed2MotorDriverTimerCallback()
{

    // ros::Time t = ros::Time::now();
    //// enable check
    catchrobo_msgs::ErrorCode error;
    sensor_msgs::JointState joint_state;
    robot_manager.getJointState(joint_state);
    enable_manager.check(joint_state, error);

    //// errorならdisable指示およびerror のpublish
    if (error.error_code != catchrobo_msgs::ErrorCode::NONE)
    {
        enableAll(false);
        enable_manager.setCurrentEnable(false);
        ros_bridge.publishError(error);
    }

    if (enable_manager.getEnable()) //// enableならtをすすめる
    {
        robot_manager.nextStep(MBED2MOTOR_DT);
    }
    else //// disableなら脱力指示
    {
        robot_manager.disable();
    }
    //// update target value
    ControlStruct control[N_MOTORS] = {};
    ControlResult::ControlResult result[N_MOTORS] = {};
    robot_manager.getMotorDrivesCommand(control, result);
    for (int i = 0; i < N_MOTORS; i++)
    {
        motor_driver_bridge.publish(control[i]);
        if (result[i] == ControlResult::FINISH)
        {
            error.id = i;
            error.error_code = catchrobo_msgs::ErrorCode::FINISH;
            ros_bridge.publishError(error);
            // ros_bridge.publishFinishFlag(i);
            // ros::Duration dt = ros::Time::now() - t;
            // ROS_INFO_STREAM("dt " << dt.toSec() );
        }
    }
}

void mbed2RosTimerCallback()
{
    //// radius
    // sensor_msgs::JointState joint_state;
    // robot_manager.getJointState(joint_state);

    std_msgs::Float32MultiArray joint_rad;
    robot_manager.getJointRad(joint_rad);
    joint_rad.data[N_MOTORS] = gripper_manager.getJointRad();
    ros_bridge.publishJointState(joint_rad);
};
void enableCallback(const catchrobo_msgs::EnableCmd &input)
{
    enable_manager.setCmd(input);
    enableAll(input.is_enable);
    enable_manager.setCurrentEnable(input.is_enable);
}

void gripperTimerCallback()
{

    ControlStruct control;
    ControlResult::ControlResult result;
    gripper_manager.getMotorDrivesCommand(control, result);
    gripper_manager.nextStep(MBED2GRIPPER_DT);
    motor_driver_bridge.publish(control);
    // ROS_INFO_STREAM("gripper id " << control.id <<" p " << control.p_des);
    // ROS_INFO_STREAM(control);
    if (result == ControlResult::FINISH)
    {
        catchrobo_msgs::ErrorCode error;
        error.id = N_MOTORS;
        error.error_code = catchrobo_msgs::ErrorCode::FINISH;
        ros_bridge.publishError(error);
        // ros_bridge.publishFinishFlag(i);
    }
}

int main(int argc, char **argv)
{
#ifndef USE_MBED
    ros::init(argc, argv, "mbed_sim");
    ros::NodeHandle nh("");
    motor_driver_bridge.setNodeHandlePtr(&nh);
    ros_bridge.setNodeHandlePtr(&nh);
#endif
    robot_manager.init(ARRIVE_THRESHOLD_RAD, FRICTION, ESTIMATE_ERROR_LIMIT_RAD);
    gripper_manager.init(GRIPPER_THRESHOLD_RAD, ESTIMATE_ERROR_LIMIT_RAD);

    ros_bridge.init(SERIAL_BAUD_RATE, rosCallback, enableCallback);

    int motor_directions[] = {-1, -1, -1, 1};
    motor_driver_bridge.init(motorDriverCallback, motor_directions);

    //// 初期値を暫定原点にする。後にROS指示で原点だしを行う
    for (size_t i = 0; i < N_MOTORS; i++)
    {
        motor_driver_bridge.setOrigin(i);
        wait(0.5);
    }

    //// motor driverへの指示開始
    Ticker ticker_motor_driver_send;
    ticker_motor_driver_send.attach(&mbed2MotorDriverTimerCallback, MBED2MOTOR_DT);

    //// ros へのフィードバック開始
    Ticker ticker;
    ticker.attach(&mbed2RosTimerCallback, MBED2ROS_DT);

    Ticker ticker_gripper;
    ticker_gripper.attach(&gripperTimerCallback, MBED2GRIPPER_DT);

    ros_bridge.spin();
}