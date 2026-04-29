#pragma once
#include "IController.h"

class PIDController : public IController {
public:
    PIDController(double kp, double ki, double kd,
                  double u_min, double u_max);

    double compute(double setpoint, double measurement, double dt) override;
    void reset() override;

private:
    double kp_;
    double ki_;
    double kd_;

    double u_min_;
    double u_max_;

    double cumulative_error_;
    double prev_error_;
};
