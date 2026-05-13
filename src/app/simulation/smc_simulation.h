#pragma once

#include "FirstOrderPlant.h"
#include "SecondOrderPlant.h"

class SMCSimulation
{
public:
    SMCSimulation(SecondOrderPlant plant, double kp, double lambda);
    double update(double setpoint, double dt);

private:
    SecondOrderPlant plant;

    double kp;
    double lambda;

    double y;
    double prev_error;
};