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
    void setPegInHoleCmd(const catchrobo_msgs::PegInHoleCmd &input)
    {
        params_ = input;
        if (!params_.run)
        {
            return;
        }
        r_ = params_.radius_delta;
        //// 半径 radius_deltaからスタートするには、以下の時刻tからスタートする必要がある
        t_ = 2.0 * pi_ * params_.radius_delta / params_.target_velocity;
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
    void getCmd(const StateStruct &z_state, catchrobo_msgs::MyRosCmd (&ros_cmd)[JOINT_NUM], ControlResult::ControlResult (&result)[JOINT_NUM])
    {
        if (!params_.run)
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
        if (z_state.position > params_.z_threshold && r_ < params_.max_radius)
        {
            ////穴に落ちるまで押し付けながらぐるぐるする
            //// x,yは円を描くように進む。速度はzのvelocity, 許容誤差はzのpositionとする。

            float omega = params_.target_velocity / r_;
            float theta = omega * t_;
            float r = params_.radius_delta * theta / (2.0 * pi_);
            r_ = r;

            float x = r * cos(theta) + params_.center_x;
            float y = r * sin(theta) * 0.5 + params_.center_y; // y軸は2倍動くため

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
            params_.run = false;
        }
    }

    int isPegInHoleMode()
    {
        return params_.run;
    }

    void nextStep(float dt)
    {
        t_ += dt;
    }

private:
    catchrobo_msgs::PegInHoleCmd params_;

    float t_;
    float r_;
    const float pi_;
};
