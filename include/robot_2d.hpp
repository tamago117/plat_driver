#pragma once

class robot_2d
{
public:
    robot_2d(){};
    robot_2d(double ini_x, double ini_y, double ini_yaw);

    double x;
    double y;
    double yaw;
};

robot_2d::robot_2d(double ini_x, double ini_y, double ini_yaw)
: x(ini_x), y(ini_y), yaw(ini_yaw)
{
    
}