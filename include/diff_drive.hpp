#pragma once

#include <arduino.h>
#include <math.h>
#include <stdint.h>
#include "robot_2d.hpp"

class diffDrive
{
public:
    //distance: distance between two tires
	diffDrive(double wheel_tred_, double wheel_radius_);
    //v:velocity,w:angular velocity
	void steering(double v, double w);
    // r_rotate, l_rotate[rad], yaw[rad]
    robot_2d odometry(double r_rotate, double l_rotate, robot_2d odom);
    robot_2d odometry_gyro(double r_rotate, double l_rotate, double yaw, robot_2d odom);
    double rightVelocity(){return vR;};
    double leftVelocity(){return vL;};

private:
    double wheel_tred;
    double wheel_radius;
    double vR;
    double vL;
};

diffDrive::diffDrive(double wheel_tred_, double wheel_radius_) : wheel_tred(wheel_tred_), wheel_radius(wheel_radius_)
{
    
}

void diffDrive::steering(double v, double w)
{
	vR = v + wheel_tred * w;
    vL = v - wheel_tred * w;
}

// r_rotate, l_rotate[rad]
robot_2d diffDrive::odometry(double r_rotate, double l_rotate, robot_2d odom)
{
    r_rotate = -r_rotate;

    // wheel rotate[rad] -> wheel move[m]
    double r_move = (r_rotate/(2*M_PI))*wheel_radius;
    double l_move = (l_rotate/(2*M_PI))*wheel_radius;

    odom.x += (r_move + l_move)/2 * cos(odom.yaw);
    odom.y += (r_move + l_move)/2 * sin(odom.yaw);
    odom.yaw += (r_move - l_move)/wheel_tred;
    
    return odom;
}

// r_rotate, l_rotate[rad], yaw[rad]
robot_2d diffDrive::odometry_gyro(double r_rotate, double l_rotate, double yaw, robot_2d odom)
{
    r_rotate = -r_rotate;

    // wheel rotate[rad] -> wheel move[m]
    double r_move = (r_rotate/(2*M_PI))*wheel_radius;
    double l_move = (l_rotate/(2*M_PI))*wheel_radius;

    odom.x += (r_move + l_move)/2 * cos(yaw);
    odom.y += (r_move + l_move)/2 * sin(yaw);
    odom.yaw = yaw;
    
    return odom;
}