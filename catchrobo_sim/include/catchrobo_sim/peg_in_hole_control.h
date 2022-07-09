#pragma once

#include "catchrobo_sim/define.h"
#include "catchrobo_sim/control_result.h"
#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>
#include <catchrobo_msgs/PegInHoleCmd.h>
#include <math.h>
//#

class PegInHoleControl
{
public:
    PegInHoleControl() : pi_(3.1415){};
    void setPegInHoleCmd(const std_msgs::Bool &input, const std_msgs::Float32MultiArray &joint_state)
    {
        float pulley_radius = 0.002 * 54 / (2 * pi_);
        radius_delta_ = 0.009 / pulley_radius;
        target_velocity_ = 2 * pi_ * radius_delta_ * 2;
        float t = 2.0;
        max_radius_ = sqrt(t * target_velocity_ / (2.0 * pi_));
        z_threshold_ = 0.05;

        center_x_ = joint_state.data[0];
        center_y_ = joint_state.data[1];
        params_ = input;
        if (!params_.data)
        {
            return;
        }
        r_ = radius_delta_;
        //// 半径 radius_deltaからスタートするには、以下の時刻tからスタートする必要がある
        t_ = 2.0 * pi_ * radius_delta_ / target_velocity_;
    }

    // void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    // {
    //     targets_[cmd.id] = cmd;

    //     ////peg in holeはz軸にパラメーターを入れて送られるとする
    //     if (cmd.id == 2)
    //     {
    //         radius_delta_ = cmd.position;
    //         target_velocity_ = cmd.velocity;
    //         z_threshold_ = cmd.acceleration_limit;
    //     }
    //     r = radius_delta_;
    //     theta = 0;
    //     // accel_curve_.reset(cmd.jerk_limit, cmd.acceleration_limit, joint_state.velocity, cmd.velocity);
    // };
    void getCmd(const StateStruct &z_state, catchrobo_msgs::MyRosCmd (&ros_cmd)[N_MOTORS], ControlResult::ControlResult (&result)[N_MOTORS])
    {
        if (!params_.data)
        {
            return;
        }

        //// 初期化
        for (size_t i = 0; i < 3; i++)
        {
            // ros_cmd[i] = targets_[i];
            ros_cmd[i].mode = catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE;
            result[i] = ControlResult::RUNNING;
        }

        //// じゃがりこが穴に入る=z軸が一定値以下になったら終了.
        //// max_radius 以上になったら諦める
        //// しきい値はz軸のposition_minとする.
        if (z_state.position > z_threshold_ && r_ < max_radius_)
        {
            ////穴に落ちるまで押し付けながらぐるぐるする
            //// x,yは円を描くように進む。速度はzのvelocity, 許容誤差はzのpositionとする。

            float omega = target_velocity_ / r_;
            float theta = omega * t_;
            float r = radius_delta_ * theta / (2.0 * pi_);
            r_ = r;

            float x = r * cos(theta) + center_x_;
            float y = r * sin(theta) * 0.5 + center_y_; // y軸は2倍動くため

            float x_vel = -r * omega * sin(theta);
            float y_vel = r * omega * cos(theta) * 0.5; // y軸は2倍動くため

            //// ros_cmdに格納
            ros_cmd[0].position = x;
            ros_cmd[0].velocity = x_vel;

            ros_cmd[1].position = y;
            ros_cmd[1].velocity = y_vel;

            //// [TODO] t_ff計算

            //// z 軸は脱力
            ros_cmd[2].kp = 0;
            ros_cmd[2].kd = 0;
            ros_cmd[2].effort = 0;
        }
        else
        {
            //穴に落ちたら終了。モーターは脱力
            for (size_t i = 0; i < 3; i++)
            {
                ros_cmd[i].mode = catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE;
                ros_cmd[i].kp = 0;
                ros_cmd[i].kd = 0;
                ros_cmd[i].effort = 0;

                result[i] = ControlResult::FINISH;
            }
            params_.data = false;
        }
    }

    int isPegInHoleMode()
    {
        return params_.data;
    }

    void nextStep(float dt)
    {
        t_ += dt;
    }

private:
    std_msgs::Bool params_;

    float t_;
    float r_;

    float radius_delta_;    //### 回転する間隔　(＝許容誤差)
    float max_radius_;      //### 最大半径
    float target_velocity_; //### 回転速度
    float z_threshold_;     //### シュート検知しきい値　(=z軸の高さ)

    float center_x_;
    float center_y_;
    const float pi_;
};
