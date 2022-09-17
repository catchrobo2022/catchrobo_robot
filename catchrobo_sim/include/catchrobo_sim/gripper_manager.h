#pragma once

#include "catchrobo_sim/define.h"
#include "catchrobo_sim/motor_manager.h"
#include "catchrobo_sim/servo_manager.h"
#include "catchrobo_sim/enable_manager.h"
#include "catchrobo_sim/obstacle_avoidance.h"
#include "motor_driver_bridge/motor_driver_struct.h"

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/MyRosCmdArray.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>

#include <vector>
#include <string>

#define TORQUE_OUTPUT
// #

class GripperManager
{
public:
    GripperManager() : motor_num_(N_MOTORS)
    {
    }

    ////本当はMyRosCmdArrayを受け取るのがキレイだが、配列要素数を取得する計算がPCとmbedで変わってしまうため、MyRosCmdで受け取るようにしている。
    void setRosCmd(const catchrobo_msgs::MyRosCmd &command)
    {
        if (command.id >= motor_num_)
        {
            servo_manager_.setRosCmd(command);
            servo_manager_.resetT();
        }
    };

    void getMotorDrivesCommand(ControlStruct &cmd, ControlResult::ControlResult &result)
    {
        servo_manager_.getCmd(cmd, result);
        ///// cmdのidを上書き。初期値である0になっている場合があるため
        cmd.id = motor_num_;
    };

    void nextStep(float dt)
    {
        servo_manager_.nextStep(dt);
    }

    float getJointRad()
    {
        StateStruct state;
        servo_manager_.getState(state);
        return state.position;
    };
    void init(float arrive_threshold, float estimate_error_limit)
    {
        servo_manager_.init(arrive_threshold, estimate_error_limit, 0);
    }

private:
    ServoManager servo_manager_;

    const int motor_num_;
};
