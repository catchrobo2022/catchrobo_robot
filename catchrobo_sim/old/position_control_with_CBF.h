#pragma once

#include "catchrobo_sim/accel_designer.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/control_result.h"

#include "motor_driver_bridge/motor_driver_struct.h"
#include <catchrobo_msgs/MyRosCmd.h>

#include <limits>

class PositionControlWithCBF
{
public:
    PositionControlWithCBF() : no_target_flag_(true), finish_already_notified_(false), temp_target_flag_(false), final_target_position_(0), dt_(0) {}
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {
        target_ = cmd;
        final_target_position_ = cmd.position;
        // setAccelDesigner(cmd, joint_state);
        no_target_flag_ = false;
        finish_already_notified_ = false;
        temp_target_flag_ = false;
    };

    // dt間隔で呼ばれる想定. except_command : 例外時に返す値。

    void getCmd(float t_, const StateStruct &state, const ControlStruct &except_command, ControlStruct &command, ControlResult::ControlResult &finished)
    {
        finished = ControlResult::RUNNING;
        if (no_target_flag_)
        {
            // まだ目標値が与えられていないとき
            command = except_command;
            return;
        }

        if (finish_already_notified_)
        {
            //// 収束後
            packAfterFinish(target_, command);
            return;
        }

        float dist = target_.position - state.position;
        float convergence_threshold_ = 0.1;
        if (fabs(dist) < convergence_threshold_)
        {
            //// 収束済み
            packAfterFinish(target_, command);
            if (temp_target_flag_)
            {
                //// 一時目標変換していたとき
                return;
            }
            finished = ControlResult::FINISH;
            finish_already_notified_ = true;
            return;
        }

        calcCBF(state.velocity, target_, state, command);
    }

    void changeTarget(bool temp_mode, float target_position, const StateStruct &joint_state)
    {
        if (temp_mode && !temp_target_flag_)
        {
            //// temp modeがONに切り替わったとき
            target_.position = target_position;
            setAccelDesigner(target_, joint_state);
        }
        else if (!temp_mode && temp_target_flag_)
        {
            //// temp modeがOFFに切り替わったとき
            target_.position = final_target_position_;
            setAccelDesigner(target_, joint_state);
        }
        temp_target_flag_ = temp_mode;
    }

    void setDt(float dt)
    {
        dt_ = dt;
    }

private:
    bool no_target_flag_;
    bool finish_already_notified_;
    bool temp_target_flag_;
    float dt_;

    ctrl::AccelDesigner accel_designer_;
    catchrobo_msgs::MyRosCmd target_;
    float final_target_position_;

    void bound(double min, double max, float &target)
    {
        if (target < min)
        {
            target = min;
        }
        if (target > max)
        {
            target = max;
        }
    }

    void setAccelDesigner(const catchrobo_msgs::MyRosCmd &cmd, const StateStruct &joint_state)
    {
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
    }

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

    void calcCBF(float current_velocity, const catchrobo_msgs::MyRosCmd target, const StateStruct &state, ControlStruct &command)
    {
        float dt = dt_;
        float dist = target_.position - state.position;
        float acceleration_limit = target.acceleration_limit;
        int sign = (dist > 0) - (dist < 0);
        float accel = sign * acceleration_limit;

        float alpha = target.jerk_limit / target.acceleration_limit;
        if (sign * current_velocity > 0)
        {
            ////　進行方向と目標値が同じで、velocity != 0のときは、ブレーキできるようCBF
            //// CBF
            if (dist > 0)
            {
                float h = dist - 0.5 * current_velocity * current_velocity / acceleration_limit;
                // ROS_INFO_STREAM("h : " << h);
                accel = acceleration_limit * (-1 + alpha * h / current_velocity);
            }
            else
            {
                float h = -dist - 0.5 * current_velocity * current_velocity / acceleration_limit;
                accel = acceleration_limit * (1 + alpha * h / current_velocity);
            }
            // ROS_INFO_STREAM("cbf : " << accel);
        }
        bound(-acceleration_limit, acceleration_limit, accel);

        float target_velocity = current_velocity + accel * dt;

        command.id = target.id;
        command.v_des = target_velocity;
        command.p_des = state.position + target_velocity * dt;
        command.torque_feed_forward = target.net_inertia * accel + target.effort;
        command.kp = target.kp;
        command.kd = target.kd;

        // ROS_INFO_STREAM("dist, velocity, accel");
        // ROS_INFO_STREAM(dist << " " << current_velocity << " " << accel);
    }
    // void packBeforeSetTargetCmd(int id, ControlStruct &cmd){
    //     cmd.id = id;
    //     cmd.p_des = 0;
    //     cmd.v_des = 0;
    //     cmd.i_ff = 0;
    //     cmd.kp = 0;
    //     cmd.kd = 0;
    // }
};
