#pragma once

#include <arduino.h>
#include <stdint.h>

class diffDrive
{
public:
    //distance: distance between two tires
	diffDrive(float distance);
    //v:velocity,w:angular velocity
	void steering(double v, double w);
    double rightVelocity(){return vR;};
    double leftVelocity(){return vL;};

private:
    float d;
    double vR;
    double vL;
};

diffDrive::diffDrive(float distance) : d(distance)
{
    
}

void diffDrive::steering(double v, double w)
{
	vR = v + d * w;
    vL = v - d * w;

}