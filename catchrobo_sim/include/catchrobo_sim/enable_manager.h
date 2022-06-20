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
    EnableManager() : current_enable_(false){};
    void setCmd(const catchrobo_msgs::EnableCmd &command)
    {
        cmd_ = command;
    };
    //// motor driver bridgeで指示を変更した際にはこの関数を読んで情報を同期させること
    void setCurrentEnable(bool is_enable)
    {
        current_enable_ = is_enable;
    };

    void check(const sensor_msgs::JointState &state, catchrobo_msgs::ErrorCode &error)
    {
        //// default
        error.error_code = catchrobo_msgs::ErrorCode::NONE;

        //// disable 状態なら何もしない
        if (!current_enable_)
        {
            return;
        }

        //// enable状態なら制約チェック

        //     // checkCollision(state, cmd_, error);
        //     checkTargetPosition(state, cmd, cmd_, error);
        //     checkOverTorque(state, cmd_, error);
        //     checkOverVelocity(state, cmd_, error);
        //     checkOverPosition(state, cmd_, error);
    }
    bool getEnable()
    {
        return current_enable_;
    }

private:
    catchrobo_msgs::EnableCmd cmd_;
    bool current_enable_;

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
};
