#pragma once

#include "catchrobo_sim/define.h"
#include "catchrobo_sim/control_result.h"
#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>
#include <math.h>
//#

class PegInHoleControl
{
public:
    PegInHoleControl() : pi_(3.1415){};
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    {
        targets_[cmd.id] = cmd;

        ////peg in holeはz軸にパラメーターを入れて送られるとする
        if (cmd.id == 2)
        {
            radius_delta_ = cmd.position;
            target_velocity_ = cmd.velocity;
            z_threshold_ = cmd.position_min;
        }
        r_ = radius_delta_;
        theta_ = 0;
        // accel_curve_.reset(cmd.jerk_limit, cmd.acceleration_limit, joint_state.velocity, cmd.velocity);
    };
    void getCmd(const StateStruct &z_state, catchrobo_msgs::MyRosCmd (&ros_cmd)[JOINT_NUM], ControlResult (&result)[JOINT_NUM])
    {

        //// 初期化
        for (size_t i = 0; i < 3; i++)
        {
            ros_cmd[i] = targets_[i];
            ros_cmd[i].mode = catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE;
            result[i] = ControlResult::RUNNING;
        }

        /** じゃがりこが穴に入る=z軸が一定値以下になったら終了.
        /* しきい値はz軸のposition_minとする.
        **/
        if (z_state.position > z_threshold_)
        {
            ////穴に落ちるまで押し付けながらぐるぐるする
            //// x,yは円を描くように進む。速度はzのvelocity, 許容誤差はzのpositionとする。
            float omega = target_velocity_ / r_;

            float x = r_ * cos(theta_);
            float y = r_ * sin(theta_) * 0.5; // y軸は2倍動くため

            float x_vel = -r_ * omega * sin(theta_);
            float y_vel = r_ * omega * cos(theta_) * 0.5; // y軸は2倍動くため

            ////次のループ用に値をすすめる
            theta_ += omega * dt_;
            if (theta_ > 2 * pi_)
            {
                //// 1周したら次の大きさの円にする
                r_ += radius_delta_;
                theta_ = 0;
            }

            //// ros_cmdに格納
            ros_cmd[0].position = x;
            ros_cmd[0].velocity = x_vel;

            ros_cmd[1].position = y;
            ros_cmd[1].velocity = y_vel;

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
        }
    }

    int isPegInHoleMode()
    {
        ////peg in hole modeかどうかはz軸で判断する
        return (targets_[2].mode == catchrobo_msgs::MyRosCmd::PEG_IN_HOLE_MODE);
    }

    //// [TODO] t_を用意する。getCmdのたびに値が変わらないようにする
    void nextStep(float dt)
    {
        dt_ = dt;
    }

private:
    catchrobo_msgs::MyRosCmd targets_[JOINT_NUM];
    double dt_;
    float r_;
    float theta_;
    float z_threshold_;
    float radius_delta_;
    float target_velocity_;
    float pi_;
    float start_x_;
    float start_y_;
};
