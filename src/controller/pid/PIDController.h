#pragma once
#include "IController.h"

class PIDController : public IController {
public:
    PIDController(double kp, double ki, double kd);

    double compute(double setpoint, double measurement, double dt) override;
    void reset() override;

private:
    double kp_;
    double ki_;
    double kd_;

    double integral_;
    double prev_error_;
};
