#include "FirstOrderPlant.h"

FirstOrderPlant::FirstOrderPlant(double K, double tau)
    : K_(K), tau_(tau), y_(0.0) {}

double FirstOrderPlant::update(double input, double dt) {
    // Discrete approximation: dy/dt = (-y + K*u)/tau
    double dy = (-y_ + K_ * input) / tau_;
    y_ += dy * dt;
    return y_;
}

void FirstOrderPlant::reset() {
    y_ = 0.0;
}
