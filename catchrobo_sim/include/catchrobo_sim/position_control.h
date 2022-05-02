#pragma once

#include "catchrobo_sim/accel_designer.h"
#include "catchrobo_sim/controller_interface.h"

#include <catchrobo_msgs/StateStruct.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/MyRosCmd.h>

#include <ros/ros.h>//ROS_INFO用

class PositionControl : public ControllerInterface
{
public:
    PositionControl():dt_(0), no_target_flag_(true), during_cal_flag_(false){};
    void init(double dt){
        dt_ = dt;
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
    };

    // dt間隔で呼ばれる想定
    void getCmd(const catchrobo_msgs::ControlStruct &old_cmd, catchrobo_msgs::ControlStruct &cmd)
    {
        t_+=dt_;
        if(t_ > accel_designer_.t_end()){
            //到達後は目標値と一致する。
            //[WARNING] トリッキーな実装：accel_designer_.t_end()はresetで値を入力するまで0を返す。
            //                         そのため、setRosCmdが呼ばれるまではこのifに入る。
            cmd = old_cmd;
            return;
        }
        packResult2Cmd(t_, accel_designer_, target_, cmd);       
        
    };

private:
    double dt_;
    double t_;
    bool no_target_flag_;
    bool during_cal_flag_;

    ctrl::AccelDesigner accel_designer_;
    catchrobo_msgs::MyRosCmd target_;

    void packResult2Cmd(double t, const ctrl::AccelDesigner &accel_designer, const catchrobo_msgs::MyRosCmd &target, catchrobo_msgs::ControlStruct &cmd){
        cmd.p_des = accel_designer.x(t);
        cmd.v_des = accel_designer.v(t);
        cmd.i_ff = accel_designer.a(t);
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
