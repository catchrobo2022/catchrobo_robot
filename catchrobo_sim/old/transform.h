#pragma once

class Transform
{
private:
    /* data */
public:
    Transform(/* args */);
    ~Transform();
    //// robot座標系での[m] -> motor回転角度[rad]に変換. gripperは入力をそのまま返す
    static float m2rad(int motor_id, float position)
    {
        const float pi = 3.141592653589;
        const float pulley_radius = 0.002 * 54.0 / (2.0 * pi);
        float ret = position / pulley_radius;
        if (motor_id == 1)
        {
            ret *= 0.5;
        }
        return ret;
    }
};

Transform::Transform(/* args */)
{
}

Transform::~Transform()
{
}
