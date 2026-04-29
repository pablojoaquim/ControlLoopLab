#pragma once
#include "IController.h"

class PIDController : public IController {
public:
    PIDController(double kp, double ki, double kd,
                  double u_min, double u_max);

    double compute(double setpoint, double measurement, double dt) override;
    void reset() override;

    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);

    double getKp() const;
    double getKi() const;
    double getKd() const;

private:
    double kp_;
    double ki_;
    double kd_;

    double u_min_;
    double u_max_;

    double cumulative_error_;
    double prev_error_;
};
