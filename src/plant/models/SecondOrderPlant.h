#pragma once
#include "IPlant.h"

class SecondOrderPlant : public IPlant {
public:
    SecondOrderPlant(double K, double omega_n, double psi);

    double update(double input, double dt) override;
    void reset() override;

private:
    double K_;
    double omega_n_;
    double psi_;

    double x1_; // output (y)
    double x2_; // velocity (dy/dt)
};
