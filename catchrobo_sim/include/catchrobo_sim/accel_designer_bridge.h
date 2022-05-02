#ifndef CATCHROBO_ACCEL_DESIGNER_BRIDGE_H
#define CATCHROBO_ACCEL_DESIGNER_BRIDGE_H

#include <sensor_msgs/JointState.h>
#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/LinearRobotControl.h>

#include "catchrobo_sim/accel_designer.h"

class AccelDesignerBridge
{
public:
    AccelDesignerBridge(int id, double dt): id_(id), dt_(dt), no_target_flag_(true){};
    void setTarget(const catchrobo_msgs::LinearRobotControl &target, const sensor_msgs::JointState &joint_state)
    {
        target_ = target;
        float start_posi = joint_state.position[id_];
        float dist = target.position[id_] - start_posi; //移動距離

        accel_designer_.reset(target.jerk_limit[id_], target.acceleration_limit[id_], target.velocity_limit[id_],
                                joint_state.velocity[id_], target.velocity[id_], dist,
                                start_posi, 0);
        t_ = 0;
        no_target_flag_ = false;
    };

    // dt間隔で呼ばれる想定
    void getCmd(catchrobo_msgs::ControlStruct &cmd)
    {
        // cmd = cmd_;
        if(no_target_flag_){
            packNoTargetCmd(id_, cmd);
            return;
        }

        t_+=dt_;
        if(t_ > accel_designer_.t_end()){
            //到達後は目標値と一致する。
            packAfterArriveCmd(id_, target_, cmd);
            return;
        }
        packResult2Cmd(id_, t_, accel_designer_, cmd);       
    };

private:
    int id_;
    double dt_;
    double t_;
    bool no_target_flag_;

    ctrl::AccelDesigner accel_designer_;
    catchrobo_msgs::LinearRobotControl target_;

    void packAfterArriveCmd(int id, const catchrobo_msgs::LinearRobotControl &target, catchrobo_msgs::ControlStruct &cmd){
        cmd.id = id;
        cmd.p_des = target_.position[id];
        cmd.v_des = target_.velocity[id];
        cmd.i_ff = target_.effort[id];
        cmd.kp = target_.kp[id];
        cmd.kd = target_.kd[id];
    }

    void packResult2Cmd(int id, double t, const ctrl::AccelDesigner &accel_designer, catchrobo_msgs::ControlStruct &cmd){
        cmd.id = id;
        cmd.p_des = accel_designer_.x(t);
        cmd.v_des = accel_designer_.v(t);
        cmd.i_ff = accel_designer_.a(t);
        cmd.kp = target_.kp[id];
        cmd.kd = target_.kd[id];
    }

    void packNoTargetCmd(int id, catchrobo_msgs::ControlStruct &cmd){
        cmd.id = id;
        cmd.p_des = 0;
        cmd.v_des = 0;
        cmd.i_ff = 0;
        cmd.kp = 0;
        cmd.kd = 0;
    }

};

#endif