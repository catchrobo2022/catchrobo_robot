#pragma once

#include "catchrobo_sim/accel_designer.h"
#include "catchrobo_sim/controller_interface.h"
#include "catchrobo_sim/safe_control.h"

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h> //ROS_INFO用

#include <limits>

class PositionControl : public ControllerInterface
{
public:
    PositionControl() : dt_(0.1), no_target_flag_(true), during_cal_flag_(false), finish_already_notified_(false){};
    void init(double dt, SafeControl &safe_control)
    {
        dt_ = dt;
        safe_control_ = safe_control;
    }
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd, const catchrobo_msgs::StateStruct &joint_state)
    {

        during_cal_flag_ = true;
        target_ = cmd;
        float start_posi = joint_state.position;
        float dist = cmd.position - start_posi; //移動距離

        accel_designer_.reset(cmd.jerk_limit, cmd.acceleration_limit, cmd.velocity_limit,
                              joint_state.velocity, cmd.velocity, dist,
                              start_posi, 0);
        t_ = 0;
        no_target_flag_ = false;
        during_cal_flag_ = false;
        finish_already_notified_ = false;
    };

    // dt間隔で呼ばれる想定. except_command : 例外時に返す値。
    void getCmd(const catchrobo_msgs::StateStruct &state, const catchrobo_msgs::ControlStruct &except_command, catchrobo_msgs::ControlStruct &command, bool &finished)
    {
        finished = false;
        if (no_target_flag_)
        {
            // まだ目標値が与えられていないとき
            command = except_command;
        }
        else
        {
            // まだ目標値が与えられた後
            t_ += dt_;
            if (t_ < accel_designer_.t_end())
            {
                //収束していないとき
                packResult2Cmd(t_, accel_designer_, target_, command);
            }
            else
            {
                //収束後
                command = except_command;
                if (!finish_already_notified_)
                {
                    finished = true;
                    finish_already_notified_ = true;
                }
            }
        }
        safe_control_.getSafeCmd(state, target_, except_command, command);
    };

private:
    double dt_;
    double t_;
    bool no_target_flag_;
    bool during_cal_flag_;
    bool finish_already_notified_;

    ctrl::AccelDesigner accel_designer_;
    catchrobo_msgs::MyRosCmd target_;
    SafeControl safe_control_;

    void packResult2Cmd(double t, const ctrl::AccelDesigner &accel_designer, const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &cmd)
    {
        cmd.id = target.id;
        cmd.p_des = accel_designer.x(t);
        cmd.v_des = accel_designer.v(t);
        cmd.i_ff = target.inertia * accel_designer.a(t) + target.effort;
        cmd.kp = target.kp;
        cmd.kd = target.kd;
    }

    // void packBeforeSetTargetCmd(int id, catchrobo_msgs::ControlStruct &cmd){
    //     cmd.id = id;
    //     cmd.p_des = 0;
    //     cmd.v_des = 0;
    //     cmd.i_ff = 0;
    //     cmd.kp = 0;
    //     cmd.kd = 0;
    // }
};
