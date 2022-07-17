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

class CheckBound
{
public:
    CheckBound() : min_(0), max_(0){};
    void init(float min, float max)
    {
        min_ = min;
        max_ = max;
    }
    bool check(float current)
    {
        bool result = true;
        if (current < min_ || max_ < current)
        {
            result = false;
        }
        return result;
    };

private:
    float min_, max_;
};

class EnableManager
{
public:
    EnableManager() : current_enable_(false), motor_num_(N_MOTORS)
    {

        // const float pi = 3.141592653589;
        // pulley_radius_ = 0.002 * 54.0 / (2.0 * pi);
        float epsilon = 0.01; // ジャストenableに引っかかることがあるので、ちょっと多めに取る
        float position_max_rad[] = {78.68526044, 25.67088441, 8.435758051};
        float position_min_rad[] = {0, -25.67088441, 0 - epsilon};
        float velocity_limit_rad[] = {180, 180, 180};
        float torque_limit_rad[] = {0.56, 0.56, 0.56};
        for (size_t i = 0; i < motor_num_; i++)
        {
            // float pos_min = m2rad(i, position_min_m[i]);
            // float pos_max = m2rad(i, position_max_m[i]);
            // float vel_min = m2rad(i, -velocity_limit_m[i]);
            // float vel_max = m2rad(i, velocity_limit_m[i]);
            // float torque_min = m2rad(i, -torque_limit_m[i]);
            // float torque_max = m2rad(i, torque_limit_m[i]);

            check_position[i].init(position_min_rad[i], position_max_rad[i]);
            check_velocity[i].init(-velocity_limit_rad[i], velocity_limit_rad[i]);
            check_torque[i].init(-torque_limit_rad[i], torque_limit_rad[i]);
        }
    };
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

        //// disable or enable_checkがF なら何もしない
        if (!current_enable_ || !cmd_.enable_check)
        {
            return;
        }

        //// enable状態なら制約チェック.一番最初に引っかかったものだけをreturn.

        for (size_t i = 0; i < motor_num_; i++)
        {
            error.id = i;
            if (!check_position[i].check(state.position[i]))
            {
                error.error_code = catchrobo_msgs::ErrorCode::OVER_POSITION;
                return;
            }

            // if (!check_velocity[i].check(state.velocity[i]))
            // {
            //     error.error_code = catchrobo_msgs::ErrorCode::OVER_VELOCITY;
            //     //                ROS_INFO_STREAM("id : " << i << "vel : " << state.velocity[i]);
            //     return;
            // }
            // if (!check_torque[i].check(state.effort[i]))
            // {
            //     error.error_code = catchrobo_msgs::ErrorCode::OVER_TORQUE;
            //     return;
            // }
        }

        // checkCollision(state, cmd_, error);
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
    CheckBound check_position[N_MOTORS];
    CheckBound check_velocity[N_MOTORS];
    CheckBound check_torque[N_MOTORS];
    bool current_enable_;
    // float pulley_radius_;
    const int motor_num_;
    //// robot座標系での[m] -> motor回転角度[rad]に変換. gripperは入力をそのまま返す
    // float m2rad(int motor_id, float position)
    // {
    //     float ret = position / pulley_radius_;
    //     if (motor_id == 1)
    //     {
    //         ret *= 0.5;
    //     }
    //     return ret;
    // }
};
