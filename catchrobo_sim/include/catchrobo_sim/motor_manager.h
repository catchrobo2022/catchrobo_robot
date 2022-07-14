#pragma once

#include "catchrobo_sim/position_control.h"
// #include "catchrobo_sim/position_control_with_CBF.h"
#include "catchrobo_sim/direct_control.h"
#include "catchrobo_sim/velocity_control.h"
#include "catchrobo_sim/safe_control.h"
#include "catchrobo_sim/go_origin_control.h"
#include "motor_driver_bridge/motor_driver_struct.h"

#include <catchrobo_msgs/MyRosCmd.h>

class MotorManager
{
public:
    MotorManager() : offset_(0), t_(0), dt_(0)
    {
        old_command_.p_des = 0;
        old_command_.v_des = 0;
        old_command_.torque_feed_forward = 0;
        old_command_.kp = 0;
        old_command_.kd = 0;

        current_state_.position = 0;
        current_state_.velocity = 0;
        current_state_.torque = 0;

        old_state_.position = 0;
        old_state_.velocity = 0;
        old_state_.torque = 0;

        double cbf_params = 1;
        safe_control_.setCBFparams(cbf_params);

        estimate_state_.position = 0;
        estimate_state_.velocity = 0;
        estimate_state_.torque = 0;
    };

    //低Hz (1 Hzとか)で呼ばれる
    void setRosCmd(const catchrobo_msgs::MyRosCmd &cmd)
    {
        ros_cmd_ = cmd;
        switch (ros_cmd_.mode)
        {
        case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:
            position_control_.setRosCmd(ros_cmd_, current_state_);
            break;
        case catchrobo_msgs::MyRosCmd::VELOCITY_CTRL_MODE:
            // direct_control_.setRosCmd(ros_cmd_, current_state_);
            velocity_control_.setRosCmd(ros_cmd_, current_state_);
            break;
        case catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE:
            // direct_control_.setRosCmd(ros_cmd_, current_state_);
            direct_control_.setRosCmd(ros_cmd_, current_state_);
            break;
        case catchrobo_msgs::MyRosCmd::GO_ORIGIN_MODE:
            go_origin_control_.setOffset(no_offset_state_, ros_cmd_, offset_);

            {
                //// [WARN] 面倒なので再帰関数
                catchrobo_msgs::MyRosCmd command;
                command.mode = catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE;
                command.kp = 0;
                command.kd = 0;
                command.effort = 0;
                setRosCmd(command);
            }
            break;

        default:
            //            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }
    };

    // dt間隔で呼ばれる. servo classではoverrideされる。
    void getCmd(ControlStruct &offset_command, ControlResult::ControlResult &result)
    {
        ControlStruct command;
        // ros::Time t_start = ros::Time::now();

        switch (ros_cmd_.mode)
        {
        case catchrobo_msgs::MyRosCmd::POSITION_CTRL_MODE:

            // position_control_.getCmd(t_, estimate_state_, old_command_, command, result);
            position_control_.getCmd(t_, current_state_, old_command_, command, result);
            safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            break;
            // case catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE:
            //     direct_control_.getCmd(current_state_, old_command_, command, result);
            //     safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            //     break;

        case catchrobo_msgs::MyRosCmd::VELOCITY_CTRL_MODE:
            velocity_control_.getCmd(current_state_, old_command_, command, result);
            safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            break;

        case catchrobo_msgs::MyRosCmd::DIRECT_CTRL_MODE:
            direct_control_.getCmd(current_state_, old_command_, command, result);
            safe_control_.getSafeCmd(current_state_, ros_cmd_, old_command_, command);
            break;

            // case catchrobo_msgs::MyRosCmd::GO_ORIGIN_MODE:
            //     ////原点だしではoffset無しの値がほしい
            //     {
            //         StateStruct no_offset_state = current_state_;
            //         no_offset_state.position += offset_;
            //         go_origin_control_.getCmd(no_offset_state, old_command_, command, result, offset_);
            //     }
            //     break;

        default:
            command = old_command_;
            //            ROS_ERROR("error : No mode in MyRosCmd");
            break;
        }
        // ros::Time now = ros::Time::now();
        // ros::Duration ros_dt = now - t_start;
        // double dt = ros_dt.toSec();
        // if (command.id == 0)
        //     ROS_INFO_STREAM(dt);
        setEstimateState(current_state_, command, estimate_state_);
        //// 基本はros座標系で行う
        old_command_ = command;
        // 最後の最後にモタドラ座標系に変換
        offset_command = command;
        offset_command.p_des += offset_;
    };

    //高Hz (500Hz)で呼ばれる
    void setCurrentState(const StateStruct &state)
    {
        no_offset_state_ = state;
        current_state_ = no_offset_state_;
        current_state_.position -= offset_;
        current_state_.velocity = (no_offset_state_.position - old_state_.position) / dt_;

        // if (current_state_.velocity > 200)
        // ROS_INFO_STREAM("dt: " << dt_ << " p: " << no_offset_state_.position << " old_p: " << old_state_.position << " v: " << current_state_.velocity);
        old_state_ = no_offset_state_;
    }

    //低Hz (50Hz)で呼ばれる
    void getState(StateStruct &state)
    {
        state = current_state_;
    }

    void getRosCmd(catchrobo_msgs::MyRosCmd &ros_cmd)
    {
        ros_cmd = ros_cmd_;
    }
    void nextStep(float dt)
    {
        dt_ = dt;
        t_ += dt;
        velocity_control_.setDt(dt);
        position_control_.setDt(dt);
    };
    bool IsPegInHoleMode()
    {
        return (ros_cmd_.mode == catchrobo_msgs::MyRosCmd::PEG_IN_HOLE_MODE);
    }

    void setObstacleInfo(bool enable, bool is_min, float limit)
    {
        safe_control_.setObstacleInfo(enable, is_min, limit);
    }

    void changeTarget(bool temp_mode, float target_position)
    {
        position_control_.changeTarget(temp_mode, target_position, current_state_);
    }

    // void changePositionMax(float max_rad)
    // {
    //     ros_cmd_.position_max = max_rad;
    // }
    void resetT()
    {
        t_ = 0;
    };

protected:
    float t_;

private:
    StateStruct current_state_;
    StateStruct no_offset_state_;
    StateStruct old_state_;
    StateStruct estimate_state_;

    ControlStruct old_command_;

    catchrobo_msgs::MyRosCmd ros_cmd_;

    //// motorの値 - offset_ = ros内での値
    float offset_;
    float dt_;
    float target_velocity_;

    PositionControl position_control_;
    // PositionControlWithCBF position_control_;
    DirectControl direct_control_;
    VelocityControl velocity_control_;
    SafeControl safe_control_;
    GoOriginControl go_origin_control_;

    void setEstimateState(const StateStruct observed_state, const ControlStruct &command, StateStruct &estimate_state)
    {
        if (command.kd > 0)
        {
            estimate_state.velocity = command.v_des;
        }
        else
        {
            estimate_state.velocity = observed_state.velocity;
        }

        if (command.kp > 0)
        {
            estimate_state.position = command.p_des;
        }
        else
        {
            estimate_state.position = observed_state.position;
        }
    }
};
