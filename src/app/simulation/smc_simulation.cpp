/*===========================================================================*/
/**
 * @file smc_simulation.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) simulation wrapper implementation.
 *
 * This module connects an IPlant implementation with an SMCController
 * instance to perform closed-loop discrete-time simulation.
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

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         SMCSimulation::SMCSimulation
 * Description  Stores the shared plant pointer, constructs the SMC controller,
 *              and initializes the simulated plant output to zero.
 *****************************************************************************/
SMCSimulation::SMCSimulation(std::shared_ptr<IPlant> plant,
                             double kp,
                             double lambda,
                             double output_min,
                             double output_max)
    : plant_(std::move(plant)),
      controller_(kp, lambda, output_min, output_max),
      y_(0.0)
{
}

/*****************************************************************************
 * Name         SMCSimulation::update
 * Description  Executes one closed-loop simulation step:
 *                1. Computes the control action using the SMC controller.
 *                2. Advances the plant model by one discrete time step.
 *                3. Returns the updated plant output.
 *****************************************************************************/
double SMCSimulation::update(double setpoint, double dt)
{
    /*===========================================================================*
     * Controller evaluation
     *===========================================================================*/
    double measurement = y_; // Current plant output is the measurement for the controller
    double u = controller_.compute(setpoint, measurement, dt);

    /*===========================================================================*
     * Plant update
     *===========================================================================*/
    y_ = plant_->update(u, dt);

    return y_;
}
