#pragma once

#include "catchrobo_sim/accel_designer.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/control_result.h"

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

#include <limits>

class PositionControl
{
public:
    PositionControl() : no_target_flag_(true), finish_already_notified_(false){};
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {

        target_ = cmd;
        float start_posi = joint_state.position;
        float target_position = cmd.position;
        // if (target_position > cmd.position_max)
        // {
        //     target_position = cmd.position_max;
        // }
        // else if (target_position < cmd.position_min)
        // {
        //     target_position = cmd.position_min;
        // }

        float dist = target_position - start_posi; //移動距離

        accel_designer_.reset(cmd.jerk_limit, cmd.acceleration_limit, cmd.velocity_limit,
                              joint_state.velocity, cmd.velocity, dist,
                              start_posi, 0);
        no_target_flag_ = false;
        finish_already_notified_ = false;
        convergence_threshold_ = 0.1;
    };

    // dt間隔で呼ばれる想定. except_command : 例外時に返す値。
    void getCmd(float dt, const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult::ControlResult &finished)
    {
        finished = ControlResult::RUNNING;
        if (no_target_flag_)
        {
            // まだ目標値が与えられていないとき
            command = except_command;
            return;
        }
        // まだ目標値が与えられた後
        if (finish_already_notified_)
        {
            packAfterFinish(target_, command);
            return;
        }

        float dist = state.position - target_.position;
        if (fabs(dist) < convergence_threshold_)
        {

            packAfterFinish(target_, command);
            finished = ControlResult::FINISH;
            finish_already_notified_ = true;
        }

        float target_acceleration = sign(dist) * target_.acceleration_limit;
        float brake_dist = 0.5 * state.velocity * state.velocity / target_acceleration;

        float target_velocity = state.velocity + sign(dist) * target_.acceleration_limit * dt;

        if (t < accel_designer_.t_end())
        {
            //収束していないとき
            packResult2Cmd(t, accel_designer_, target_, command);
        }
        else
        {
            //収束後
            packAfterFinish(target_, command);
            // command = except_command;
            if (!finish_already_notified_)
            {
                finished = ControlResult::FINISH;
                finish_already_notified_ = true;
            }
        }
    };

private:
    bool no_target_flag_;
    bool finish_already_notified_;
    float convergence_threshold_;

    ctrl::AccelDesigner accel_designer_;
    catchrobo_msgs::MyRosCmd target_;

    void packResult2Cmd(double t, const ctrl::AccelDesigner &accel_designer, const catchrobo_msgs::MyRosCmd &target, ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = accel_designer.x(t);
        cmd.v_des = accel_designer.v(t);
        cmd.torque_feed_forward = target.net_inertia * accel_designer.a(t) + target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }

    void packAfterFinish(const catchrobo_msgs::MyRosCmd &target, ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = target.position;
        cmd.v_des = target.velocity;
        cmd.torque_feed_forward = target.net_inertia * 0 + target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }

    // void packBeforeSetTargetCmd(int id, ControlStruct &cmd){
    //     cmd.id = id;
    //     cmd.p_des = 0;
    //     cmd.v_des = 0;
    //     cmd.i_ff = 0;
    //     cmd.kp = 0;
    //     cmd.kd = 0;
    // }

    int sign(float num)
    {
        return (num > 0) - (num < 0);
    }
};
