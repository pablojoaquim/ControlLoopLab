#include "FirstOrderPlant.h"

FirstOrderPlant::FirstOrderPlant(double K, double tau)
    : K_(K), tau_(tau), y_(0.0) {}

double FirstOrderPlant::update(double input, double dt) {
    // dy/dt = (-y + K*u)/tau = m (slope at current state (Euler method))
    double m = (-y_ + K_ * input) / tau_;
    y_ = y_ + m * dt;
    return y_;
}

void FirstOrderPlant::reset() {
    y_ = 0.0;
}
