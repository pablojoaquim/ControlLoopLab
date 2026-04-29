#include "SecondOrderPlant.h"

SecondOrderPlant::SecondOrderPlant(double K, double omega_n, double psi)
    : K_(K), omega_n_(omega_n), psi_(psi),
      x1_(0.0), x2_(0.0) {}

double SecondOrderPlant::update(double input, double dt) {
    // dx1/dt = x2
    double dx1 = x2_;

    // dx2/dt = -2*psi*omega_n*x2 - omega_n^2*x1 + K*omega_n^2*u
    double dx2 = -2.0 * psi_ * omega_n_ * x2_
               - omega_n_ * omega_n_ * x1_
               + K_ * omega_n_ * omega_n_ * input;

    // Euler integration
    x1_ += dx1 * dt;
    x2_ += dx2 * dt;

    return x1_;
}

void SecondOrderPlant::reset() {
    x1_ = 0.0;
    x2_ = 0.0;
}
