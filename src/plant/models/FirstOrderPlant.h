#pragma once
#include "IPlant.h"

class FirstOrderPlant : public IPlant {
public:
    FirstOrderPlant(double K, double tau);

    double update(double input, double dt) override;
    void reset() override;

private:
    double K_;     // gain
    double tau_;   // time constant
    double y_;     // current output
};
