/*===========================================================================*/
/**
 * @file PIDController.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * PID controller implementation with output saturation and anti-windup.
 *
 * @section ABBR ABBREVIATIONS:
 *   - PID - Proportional–Integral–Derivative
 *   - kp  - Proportional gain
 *   - ki  - Integral gain
 *   - kd  - Derivative gain
 *   - dt  - Discrete time step (seconds)
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
#include "PIDController.h"
#include <algorithm>

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
 * Name         PIDController::PIDController
 * Description  Constructs the controller with the given gains and output limits,
 *              and zeroes the internal integrator and previous-error state.
 *****************************************************************************/
PIDController::PIDController(double kp, double ki, double kd,
                             double u_min, double u_max)
    : kp_(kp), ki_(ki), kd_(kd),
      u_min_(u_min), u_max_(u_max),
      cumulative_error_(0.0), prev_error_(0.0) {}

/*****************************************************************************
 * Name         PIDController::compute
 * Description  Evaluates the PID control law for the current time step.
 *              Anti-windup is implemented by freezing the integrator whenever
 *              the unclamped output exceeds the saturation limits.
 *****************************************************************************/
double PIDController::compute(double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    /* Derivative: rate of error change */
    double error_rate = (error - prev_error_) / dt;

    /* Proportional term */
    double p = kp_ * error;

    /* Tentative integral update */
    double i = cumulative_error_ + error * dt;

    /* Derivative term */
    double d = kd_ * error_rate;

    /* Unclamped output */
    double u = p + ki_ * i + d;

    /* Apply output saturation */
    double u_sat = std::clamp(u, u_min_, u_max_);

    /* Anti-windup: only commit the integral update when not saturating */
    if (u == u_sat)
    {
        cumulative_error_ = i;
    }

    prev_error_ = error;

    return u_sat;
}

/*****************************************************************************
 * Name         PIDController::reset
 * Description  Resets the integrator accumulator and previous error to zero.
 *****************************************************************************/
void PIDController::reset()
{
    cumulative_error_ = 0.0;
    prev_error_ = 0.0;
}

/*****************************************************************************
 * Name         PIDController::setKp
 * Description  Updates the proportional gain.
 *****************************************************************************/
void PIDController::setKp(double kp)
{
    kp_ = kp;
}

/*****************************************************************************
 * Name         PIDController::setKi
 * Description  Updates the integral gain. The cumulative error is rescaled
 *              so that the existing integral contribution remains unchanged
 *              after the gain change.
 *****************************************************************************/
void PIDController::setKi(double ki)
{
    if (ki_ != 0.0)
    {
        /* Scale cumulative error to maintain the same integral contribution */
        cumulative_error_ *= (ki_ / ki);
    }
    ki_ = ki;
}

/*****************************************************************************
 * Name         PIDController::setKd
 * Description  Updates the derivative gain.
 *****************************************************************************/
void PIDController::setKd(double kd)
{
    kd_ = kd;
}

/*****************************************************************************
 * Name         PIDController::getKp
 * Description  Returns the current proportional gain.
 *****************************************************************************/
double PIDController::getKp() const { return kp_; }

/*****************************************************************************
 * Name         PIDController::getKi
 * Description  Returns the current integral gain.
 *****************************************************************************/
double PIDController::getKi() const { return ki_; }

/*****************************************************************************
 * Name         PIDController::getKd
 * Description  Returns the current derivative gain.
 *****************************************************************************/
double PIDController::getKd() const { return kd_; }
