#include "PIDController.h"
#include <algorithm> // std::clamp

PIDController::PIDController(double kp, double ki, double kd,
                             double u_min, double u_max)
    : kp_(kp), ki_(ki), kd_(kd),
      u_min_(u_min), u_max_(u_max),
      cumulative_error_(0.0), prev_error_(0.0) {}

double PIDController::compute(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    // Derivative (rate of error change)
    double error_rate = (error - prev_error_) / dt;

    // Proportional
    double p = kp_ * error;

    // Integral (tentative update)
    double i = cumulative_error_ + error * dt;

    // Derivative term
    double d = kd_ * error_rate;

    // Compute unclamped output
    double u = p + ki_ * i + d;

    // Apply saturation
    double u_sat = std::clamp(u, u_min_, u_max_);

    // Anti-windup: only integrate if not saturating
    if (u == u_sat) {
        cumulative_error_ = i;
    }

    prev_error_ = error;

    return u_sat;
}

void PIDController::reset() {
    cumulative_error_ = 0.0;
    prev_error_ = 0.0;
}
