#pragma once

#include <memory>

#include "FirstOrderPlant.h"
#include "SecondOrderPlant.h"

class SMCSimulation
{
public:
    SMCSimulation(std::shared_ptr<IPlant> plant, double kp, double lambda);
    double update(double setpoint, double dt);

private:
    std::shared_ptr<IPlant> plant;

    double kp;
    double lambda;

    double y;
    double prev_error;
};