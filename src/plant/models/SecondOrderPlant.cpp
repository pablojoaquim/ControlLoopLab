/*===========================================================================*/
/**
 * @file SecondOrderPlant.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Second-order plant model implementation using Euler integration.
 *
 * @section ABBR ABBREVIATIONS:
 *   - K       - Static gain
 *   - omega_n - Natural frequency (rad/s)
 *   - psi     - Damping ratio
 *
 * @section TRACE TRACEABILITY INFO:
 *   - Design Document(s):
 *     - @todo Update list of design document(s).
 *
 *   - Requirements Document(s):
 *     - @todo Update list of requirements document(s)
 *
 *   - Applicable Standards (in order of precedence: highest first):
 *     - @todo Update list of other applicable standards
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/
#include "SecondOrderPlant.h"

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Variables Definitions
 *===========================================================================*/

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/

/*===========================================================================*
 * Local Inline Function Definitions and Function-Like Macros
 *===========================================================================*/

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         SecondOrderPlant::SecondOrderPlant
 * Description  Constructs the plant with the given dynamic parameters and
 *              zeroes the internal state variables.
 *****************************************************************************/
SecondOrderPlant::SecondOrderPlant(double K, double omega_n, double psi)
    : K_(K), omega_n_(omega_n), psi_(psi),
      x1_(0.0), x2_(0.0) {}

/*****************************************************************************
 * Name         SecondOrderPlant::update
 * Description  Advances the plant state by one time step using the Euler method.
 *              The state-space equations are:
 *                dx1/dt = x2
 *                dx2/dt = -2*psi*omega_n*x2 - omega_n^2*x1 + K*omega_n^2*u
 *****************************************************************************/
double SecondOrderPlant::update(double input, double dt)
{
    double dx1 = x2_;
    double dx2 = -2.0 * psi_ * omega_n_ * x2_
                 - omega_n_ * omega_n_ * x1_
                 + K_ * omega_n_ * omega_n_ * input;

    /* Euler integration */
    x1_ += dx1 * dt;
    x2_ += dx2 * dt;

    return x1_;
}

/*****************************************************************************
 * Name         SecondOrderPlant::reset
 * Description  Resets both state variables to zero.
 *****************************************************************************/
void SecondOrderPlant::reset()
{
    x1_ = 0.0;
    x2_ = 0.0;
}
