#pragma once

#include "catchrobo_sim/transform.h"
#include "catchrobo_sim/motor_manager.h"
#include "motor_driver_bridge/motor_driver_struct.h"
#include "define.h"

struct Obstacle
{
    float edge[3][2]; // x(min, max),y(min, rad),z(min, max) の順. edge[0][1]でx軸のmaxが手に入る
};

class ObstacleAvoidance
{
private:
    Obstacle obstacles_rad[1];
    void avoidSurface(MotorManager (&motor_manager_)[N_MOTORS])
    {

        int obstacle_num = 0;
        bool aside_obstacle[] = {false, false, false}; //// 現在値のi軸成分がが障害物のi座標に入っていればtrue. □ * の位置関係にいるときは、y軸がtrueとなる.
        for (size_t i = 0; i < N_MOTORS; i++)
        {
            StateStruct state;
            motor_manager_[i].getState(state);
            float position = state.position;
            float obstacle_min = obstacles_rad[obstacle_num].edge[i][0];
            float obstacle_max = obstacles_rad[obstacle_num].edge[i][1];
            if (obstacle_min < position && position < obstacle_max)
            {
                aside_obstacle[i] = true;
            }

            //// obstacle　infoの初期化
            motor_manager_[i].setObstacleInfo(false, 0, 0); //// 後ろ２つの引数はdummy.使われない
        }

        // 障害物に当たりそうならまずzを動かす
        if (aside_obstacle[0] && aside_obstacle[1])
        {
            ////障害物のz軸方向(上空or下)にいるとき.
            //// z軸の制限を変更
            int change_axis = 2;
            changePositionLimitAsideObstacle(change_axis, obstacle_num, motor_manager_);
            // ROS_INFO_STREAM("z obstacle");
        }
        else if (aside_obstacle[2] && aside_obstacle[0])
        {
            //// 障害物のy軸方向にいるとき,
            //// y軸の制限を変更
            int change_axis = 1;
            changePositionLimitAsideObstacle(change_axis, obstacle_num, motor_manager_);
            // ROS_INFO_STREAM("y obstacle");
        }
        else if (aside_obstacle[1] && aside_obstacle[2])
        {
            int change_axis = 0;
            changePositionLimitAsideObstacle(change_axis, obstacle_num, motor_manager_);
            // ROS_INFO_STREAM("x obstacle");
        }
    }

    void changePositionLimitAsideObstacle(int change_axis, int obstacle_num, MotorManager (&motor_manager_)[N_MOTORS])
    {

        StateStruct state;
        motor_manager_[change_axis].getState(state);

        float position = state.position;
        float obstacle_min = obstacles_rad[obstacle_num].edge[change_axis][0];
        float obstacle_max = obstacles_rad[obstacle_num].edge[change_axis][1];
        bool near_min = fabs(position - obstacle_min) < fabs(position - obstacle_max);
        if (near_min)
        {
            motor_manager_[change_axis].setObstacleInfo(true, false, obstacle_min);
        }
        else
        {
            motor_manager_[change_axis].setObstacleInfo(true, true, obstacle_max);
        }
    }

    void noCollisionPlan(MotorManager (&motor_manager_)[N_MOTORS])
    {
        int obstacle_num = 0;

        catchrobo_msgs::MyRosCmd ros_cmd[N_MOTORS];
        for (size_t i = 0; i < N_MOTORS; i++)
        {
            motor_manager_[i].getRosCmd(ros_cmd[i]);
        }

        float z_top = obstacles_rad[obstacle_num].edge[2][1];
        //// 下から上に向かうときは何も変えなくて大丈夫
        if (ros_cmd[2].position > z_top)
        {
            return;
        }
        //// 下が目標値のときは壁より下がりすぎないよう注意

        //// 壁をまたぐかチェック
        if (isOverHill(ros_cmd[0].position, ros_cmd[1].position, obstacles_rad[obstacle_num]))
        {
            motor_manager_[2].setObstacleInfo(true, true, z_top);
        }
    }

    bool isOverHill(float target_x, float target_y, Obstacle &obstacle)
    {
        ////丘をまたぐとき
        // return true

        //// でなければ
        return false;
    }

public:
    ObstacleAvoidance(/* args */);
    ~ObstacleAvoidance();

    void changePositionLimit(MotorManager (&motor_manager_)[N_MOTORS])
    {
        avoidSurface(motor_manager_);
        noCollisionPlan(motor_manager_);
    }
};

ObstacleAvoidance::ObstacleAvoidance(/* args */)
{
    //// x軸上の配線との干渉を防ぐ
    float edge_m[3][2] = {{-1000, 1.1}, {-0.15, 0.15}, {-1000, 0.12}};

    // radに変換
    for (int axis = 0; axis < 3; axis++)
    {
        for (int i = 0; i < 2; i++)
        {
            obstacles_rad[0].edge[axis][i] = Transform::m2rad(axis, edge_m[axis][i]);
        }
    }
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}
