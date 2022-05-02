#ifndef CATCHROBO_CONTROLLER_H
#define CATCHROBO_CONTROLLER_H

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/LinearRobotControl.h>

#include "catchrobo_sim/accel_designer.h"

class Controller
{
public:
    Controller()
    {
        for (size_t i = 0; i < 3; i++)
        {
            cmd_[i].id = i;
        }
    };
    void setTarget(const catchrobo_msgs::LinearRobotControl &target, const sensor_msgs::JointState &joint_state)
    {
        for (size_t i = 0; i < 3; i++)
        {
            cmd_[i].p_des = target.position[i];

            // float start_posi = joint_state.position[i];
            // float dist = target.position[i] - start_posi; //移動距離

            // accel_designer_.reset(target.jerk_limit[i], target.acceleration_limit[i], target.velocity_limit[i],
            //                       joint_state.velocity[i], target.velocity[i], dist,
            //                       start_posi, 0);
        }

        // joint_state_ = joint_state;

    };
    void getCmd(int id, catchrobo_msgs::ControlStruct &cmd)
    {
        cmd = cmd_[id];
    };

private:
    catchrobo_msgs::ControlStruct cmd_[3];

    sensor_msgs::JointState joint_state_;

    // ctrl::AccelDesigner accel_designer_;
};

#endif