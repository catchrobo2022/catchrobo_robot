#pragma once

typedef struct
{
    int id;
    float position, velocity, torque;
} StateStruct;

typedef struct
{
    int id;
    float p_des, v_des, torque_feed_forward, kp, kd;
} ControlStruct;