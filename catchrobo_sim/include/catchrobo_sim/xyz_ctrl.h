#ifndef CATCHROBO_XYZ_CTRL_H
#define CATCHROBO_XYZ_CTRL_H

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/LinearRobotControl.h>

#include "catchrobo_sim/accel_designer_bridge.h"

class XYZCtrl
{
public:
    XYZCtrl(double dt): accel_designer_{AccelDesignerBridge(0, dt), AccelDesignerBridge(1, dt), AccelDesignerBridge(2, dt)}{};
    void setTarget(const catchrobo_msgs::LinearRobotControl &target, const sensor_msgs::JointState &joint_state)
    {
        for (size_t i = 0; i < 3; i++)
        {
            accel_designer_[i].setTarget(target, joint_state);
        }

    };
    void getCmd(int id, catchrobo_msgs::ControlStruct &cmd)
    {
        accel_designer_[id].getCmd(cmd);
    };

private:
    AccelDesignerBridge accel_designer_[3];
};

#endif