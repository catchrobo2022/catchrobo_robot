//#define USE_MBED

#ifdef USE_MBED
#include "mbed.h"
#include "motor_driver_bridge/motor_driver_bridge_mbed.h"
#include "catchrobo_sim/ros_bridge_mbed.h"
#else
#include <ros/ros.h>
#include "sim_only/motor_driver_bridge_sim.h"
#include "sim_only/ros_bridge_sim.h"
#include "sim_only/ticker_sim.h"
#endif

#include "catchrobo_sim/robot_manager.h"

const float MBED2ROS_DT = 0.01;  // 10Hz
const float MBED2MOTOR_DT = 0.2; // 500Hz
const int SERIAL_BAUD_RATE = 115200;

MotorDriverBridge motor_driver_bridge;
RosBridge ros_bridge;
RobotManager robot_manager;

void motorDriverCallback(const StateStruct &input)
{
    robot_manager.setCurrentState(input);
};

void rosCallback(const catchrobo_msgs::MyRosCmd &command)
{
    robot_manager.setRosCmd(command);
}

void mbed2MotorDriverTimerCallback()
{
    bool is_enable;
    bool change_enable;
    catchrobo_msgs::ErrorCode error;
    ControlResult result[JOINT_NUM];
    ControlStruct control[JOINT_NUM];
    robot_manager.getMotorDrivesCommand(is_enable, change_enable, error, control, result);
    // ROS_INFO_STREAM(is_enable << " " << change_enable << " " << error);
    /////enable, disableが切り替わるときは、enable指示のみを出してreturn
    if (!is_enable)
    {
        //// disableのとき
        if (change_enable)
        {
            for (int i = 0; i < N_MOTORS; i++)
            {
                motor_driver_bridge.disableMotor(i);
            }
            ros_bridge.publishError(error);
        }
        return;
    }
    //// enableのとき
    //// 切り替わりタイミングならモーター励起
    if (change_enable)
    {
        for (int i = 0; i < N_MOTORS; i++)
        {
            motor_driver_bridge.enableMotor(i);
        }
        return;
    }

    /////enable後
    robot_manager.nextStep(MBED2MOTOR_DT);

    //// update target value
    for (int i = 0; i < N_MOTORS; i++)
    {

        motor_driver_bridge.publish(control[i]);

        switch (result[i])
        {
        case ControlResult::RUNNING:
            break;
        case ControlResult::FINISH:
            ros_bridge.publishFinishFlag(i);
            break;
        case ControlResult::SET_ORIGIN:
            motor_driver_bridge.setOrigin(i);
            ros_bridge.publishFinishFlag(i);
            break;
        default:
            break;
        }
    }
}

void mbed2RosTimerCallback()
{
    //// radius
    sensor_msgs::JointState joint_state;
    robot_manager.getJointState(joint_state);
    ros_bridge.publishJointState(joint_state);
};

// void enableCallback(const std_msgs::Bool &input)
// {
//     if (input.data)
//     {
//         for (int i = 0; i < N_MOTORS; i++)
//         {
//             motor_driver_bridge.enableMotor(i);
//         }
//     }
//     else
//     {
//         for (int i = 0; i < N_MOTORS; i++)
//         {
//             motor_driver_bridge.disableMotor(i);
//         }
//     }
// }
void enableCallback(const catchrobo_msgs::EnableCmd &input)
{
    robot_manager.setEnableParams(input);
}

int main(int argc, char **argv)
{
#ifndef USE_MBED
    ros::init(argc, argv, "mbed_sim");
    ros::NodeHandle nh("");
    motor_driver_bridge.setNodeHandlePtr(&nh);
    ros_bridge.setNodeHandlePtr(&nh);
#endif

    ros_bridge.init(SERIAL_BAUD_RATE, rosCallback, enableCallback);
    motor_driver_bridge.init(motorDriverCallback);

    Ticker ticker_motor_driver_send;
    ticker_motor_driver_send.attach(&mbed2MotorDriverTimerCallback, MBED2MOTOR_DT);

    Ticker ticker;
    ticker.attach(&mbed2RosTimerCallback, MBED2ROS_DT);
    ros_bridge.spin();
}