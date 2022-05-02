#ifndef CATCHROBO_CONTROLLER_BASIC_H
#define CATCHROBO_CONTROLLER_BASIC_H

#include <catchrobo_msgs/ControlStruct.h>
#include <catchrobo_msgs/LinearRobotControl.h>

class Controller
{
public:

    Controller(){
        for (size_t i = ; i < 3; i++)
        {
            cmd_[i].id = i;
        }
    };
    void setTarget(const catchrobo_msgs::LinearRobotControl &target){
        for (size_t i = 0; i < 3; i++)
        {
            cmd_[i].p_des = target.position[i];
        }
        
    };
    void getCmd(int id, catchrobo_msgs::ControlStruct &cmd){
        cmd = cmd_[id];
    };
private:
    catchrobo_msgs::ControlStruct cmd_[3];
};


#endif