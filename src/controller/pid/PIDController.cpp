#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0), prev_error_(0.0) {}

double PIDController::compute(double setpoint, double measurement, double dt) {
    double error = setpoint - measurement;

    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;

    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}
