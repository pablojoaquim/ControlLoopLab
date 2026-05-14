/*===========================================================================*/
/**
 * @file pid_simulation.cpp
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
#include "pid_simulation.h"
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
 * Name         PIDSimulation::PIDSimulation
 * Description  Stores the shared plant pointer and PID parameters, and zeroes
 *              the output state and previous error.
 *****************************************************************************/
PIDSimulation::PIDSimulation(std::shared_ptr<IPlant> plant, double kp, double ki, double kd, double output_min, double output_max)
    : plant_(std::move(plant)),
      y_(0.0),
      prev_error_(0.0),
      pid_(kp, ki, kd, output_min, output_max)
{
}

/*****************************************************************************
 * Name         PIDSimulation::update
 * Description  Executes one closed-loop PID step:
 *                1. Computes the tracking error and its finite-difference
 *                   derivative.
 *                2. Evaluates the PID control action.
 *                3. Advances the plant by one dt step and returns the output.
 *****************************************************************************/
double PIDSimulation::update(double setpoint, double dt)
{
    /*===========================================================================*
     * PID control signal computation
     *===========================================================================*/
    double measurement = y_; // Current plant output is the measurement for the controller
    double u = pid_.compute(setpoint, measurement, dt);
    
    /*===========================================================================*
     * Plant update
     *===========================================================================*/
    y_ = plant_->update(u, dt);
    return y_;
}