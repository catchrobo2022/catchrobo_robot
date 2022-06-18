#pragma once

#include "catchrobo_sim/accel_curve.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/control_result.h"
#include "motor_driver_bridge/motor_driver_struct.h"
#include "catchrobo_sim/define.h"

#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/EnableCmd.h>
#include <catchrobo_msgs/ErrorCode.h>
#include <sensor_msgs/JointState.h>

//// ブラシレスしかチェックしない

// struct EnableParams
// {
// public:
//     float position_min[4];
//     float position_max[4];
//     float velocity_limit[4];
//     float torque_limit[4];
//     float trajectory_error_limit[4];
//     EnableParams(){};
// }

class EnableManager
{
public:
    EnableManager() : already_enable_(false){};
    void setParams(const catchrobo_msgs::EnableCmd &command)
    {
        params_ = command;
    };

    void check(const sensor_msgs::JointState &state, const ControlStruct (&cmd)[JOINT_NUM], bool &is_enable, bool &change_enable, catchrobo_msgs::ErrorCode &error)
    {
        if (params_.is_enable)
        {
            //// enable指示
            error.error_code = catchrobo_msgs::ErrorCode::NONE;
            // if (params_.enable_check)
            // {

            //     // checkCollision(state, params_, error);
            //     checkTargetPosition(state, cmd, params_, error);
            //     checkOverTorque(state, params_, error);
            //     checkOverVelocity(state, params_, error);
            //     checkOverPosition(state, params_, error);
            // }

            is_enable = false;
            if (error.error_code == catchrobo_msgs::ErrorCode::NONE)
            {
                is_enable = true;
            }

            setEnable(is_enable, change_enable, already_enable_, params_);
        }
        else
        {
            is_enable = false;
            setEnable(is_enable, change_enable, already_enable_, params_);
        }
    }

private:
    catchrobo_msgs::EnableCmd params_;
    bool already_enable_;

    // void checkOverPosition(const sensor_msgs::JointState &state, const catchrobo_msgs::EnableCmd params, catchrobo_msgs::ErrorCode &error)
    // {
    //     for (int i = 0; i < N_MOTORS; i++)
    //     {
    //         if (state.position[i] < params.position_min[i] || state.position[i] > params.position_max[i])
    //         {
    //             error.id = i;
    //             error.error_code = catchrobo_msgs::ErrorCode::OVER_POSITION;
    //             return;
    //         }
    //     }
    // }

    // void checkOverVelocity(const sensor_msgs::JointState &state, const catchrobo_msgs::EnableCmd params, catchrobo_msgs::ErrorCode &error)
    // {
    //     for (int i = 0; i < N_MOTORS; i++)
    //     {
    //         if (fabs(state.velocity[i]) > params.velocity_limit[i])
    //         {
    //             error.id = i;
    //             error.error_code = catchrobo_msgs::ErrorCode::OVER_VELOCITY;
    //             return;
    //         }
    //     }
    // }

    // void checkOverTorque(const sensor_msgs::JointState &state, const catchrobo_msgs::EnableCmd params, catchrobo_msgs::ErrorCode &error)
    // {
    //     for (int i = 0; i < N_MOTORS; i++)
    //     {
    //         if (fabs(state.effort[i]) > params.torque_limit[i])
    //         {
    //             error.id = i;
    //             error.error_code = catchrobo_msgs::ErrorCode::OVER_TORQUE;
    //             return;
    //         }
    //     }
    // }

    // void checkTargetPosition(const sensor_msgs::JointState &state, const ControlStruct (&cmd)[JOINT_NUM], const catchrobo_msgs::EnableCmd params, catchrobo_msgs::ErrorCode &error)
    // {
    //     for (int i = 0; i < N_MOTORS; i++)
    //     {
    //         if (fabs(cmd[i].kp) > 0)
    //         {
    //             if (fabs(state.position[i] - cmd[i].p_des) > params.trajectory_error_limit[i])
    //             {
    //                 error.id = i;
    //                 error.error_code = catchrobo_msgs::ErrorCode::FAR_TARGET_POSITION;
    //                 return;
    //             }
    //         }
    //     }
    // }

    // void checkCollision(const sensor_msgs::JointState &state, const catchrobo_msgs::EnableCmd params, catchrobo_msgs::ErrorCode &error)
    // {
    //     for (int i = 0; i < N_MOTORS; i++)
    //     {
    //         //// 1軸でもobstacle外なら衝突はしていない
    //         if (state.position[i] < params.obstacle_min[i] || state.position[i] > params.obstacle_max[i])
    //         {
    //             return;
    //         }
    //     }
    //     error.error_code = catchrobo_msgs::ErrorCode::COLLISION;
    // }

    void setEnable(bool is_enable, bool &change_enable, bool &already_enable, catchrobo_msgs::EnableCmd &params)
    {
        params_.is_enable = is_enable;
        if (is_enable == already_enable)
        {
            change_enable = false;
        }
        else
        {
            change_enable = true;
        }
        already_enable = is_enable;
    }
};
