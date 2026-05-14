/*===========================================================================*/
/**
 * @file SMCController.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) implementation with output saturation.
 *
 * @section ABBR ABBREVIATIONS:
 *   - SMC    - Sliding Mode Control
 *   - lambda - Sliding surface coefficient
 *   - eta    - Switching gain
 *   - dt     - Discrete time step (seconds)
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
#include "SMCController.h"
#include <algorithm>

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         SMCController::SMCController
 * Description  Constructs the controller with the given sliding surface
 *              parameters and output limits.
 *****************************************************************************/
SMCController::SMCController(double kp,
                             double lambda,
                             double u_min,
                             double u_max)
    : kp_(kp),
      lambda_(lambda),
      u_min_(u_min),
      u_max_(u_max),
      prev_error_(0.0)
{
}

/*****************************************************************************
 * Name         SMCController::compute
 * Description  Evaluates the Sliding Mode Control law for the current time
 *              step.
 *
 *              Sliding surface:
 *                  s = lambda * error + error_rate
 *
 *              Control law:
 *                  u = kp * sign(s)
 *****************************************************************************/
double SMCController::compute(double setpoint,
                              double measurement,
                              double dt)
{
    double error = setpoint - measurement;
    double d_error = (error - prev_error_) / dt;

    /*===========================================================================*
     * Sliding surface calculation
     *===========================================================================*/
    double s = error + (lambda_ * d_error);
    prev_error_ = error;

    /*===========================================================================*
     * Sliding Mode Control law
     *===========================================================================*/
    double u = kp_ * (s > 0.0 ? 1.0 : -1.0);

    /* Output saturation */
    u = std::clamp(u, u_min_, u_max_);

    return u;
}

/*****************************************************************************
 * Name         SMCController::reset
 * Description  Resets the previous error state to zero.
 *****************************************************************************/
void SMCController::reset()
{
    prev_error_ = 0.0;
}

/*****************************************************************************
 * Name         SMCController::setLambda
 * Description  Updates the sliding surface coefficient.
 *****************************************************************************/
void SMCController::setLambda(double lambda)
{
    lambda_ = lambda;
}

/*****************************************************************************
 * Name         SMCController::setKp
 * Description  Updates the switching gain.
 *****************************************************************************/
void SMCController::setKp(double kp)
{
    kp_ = kp;
}

/*****************************************************************************
 * Name         SMCController::getLambda
 * Description  Returns the current sliding surface coefficient.
 *****************************************************************************/
double SMCController::getLambda() const
{
    return lambda_;
}

/*****************************************************************************
 * Name         SMCController::getKp
 * Description  Returns the current switching gain.
 *****************************************************************************/
double SMCController::getKp() const
{
    return kp_;
}
