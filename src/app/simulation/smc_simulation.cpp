/*===========================================================================*/
/**
 * @file SMCSimulation.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) simulation wrapper implementation.
 *
 * @section ABBR ABBREVIATIONS:
 *   - SMC  - Sliding Mode Controller
 *   - dt   - Discrete time step (s)
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
#include "smc_simulation.h"
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
 * Name         SMCSimulation::SMCSimulation
 * Description  Stores the shared plant pointer and SMC parameters, and zeroes
 *              the output state and previous error.
 *****************************************************************************/
SMCSimulation::SMCSimulation(std::shared_ptr<IPlant> plant, double kp, double lambda)
    : plant_(std::move(plant)),
      kp_(kp),
      lambda_(lambda),
      y_(0.0),
      prev_error_(0.0)
{
}

/*****************************************************************************
 * Name         SMCSimulation::update
 * Description  Executes one closed-loop SMC step:
 *                1. Computes the tracking error and its finite-difference
 *                   derivative.
 *                2. Evaluates the sliding surface s = error + lambda * d_error.
 *                3. Applies the sliding mode control law u = kp * sign(s).
 *                4. Advances the plant by one dt step and returns the output.
 *****************************************************************************/
double SMCSimulation::update(double setpoint, double dt)
{
    /*===========================================================================*
     * Sliding surface calculation
     *===========================================================================*/
    double error   = setpoint - y_;
    double d_error = (error - prev_error_) / dt;
    double s       = error + (lambda_ * d_error);
    prev_error_    = error;

    /*===========================================================================*
     * Sliding Mode Control law
     *===========================================================================*/
    double u = kp_ * (s > 0.0 ? 1.0 : -1.0);

    /*===========================================================================*
     * Plant update
     *===========================================================================*/
    y_ = plant_->update(u, dt);
    return y_;
}