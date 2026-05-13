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
 * Sliding Mode Controller (SMC) simulation module
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/
#include "smc_simulation.h"
#include <algorithm>

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/****************************************************************************
 * @fn         SMCSimulation
 * @brief      Constructor
 * @param[in]  plant   The plant model to control
 * @param[in]  kp      Proportional gain for SMC
 * @param[in]  lambda  Tuning parameter for sliding surface
 *****************************************************************************/
SMCSimulation::SMCSimulation(SecondOrderPlant plant, double kp, double lambda)
: plant(plant), kp(kp), lambda(lambda), y(0.0), prev_error(0.0)
{
}

/****************************************************************************
 * @fn         update
 * @brief      Executes one simulation step
 * @param[in]  setpoint Desired reference
 * @param[in]  dt       Time step
 * @return     Current plant output after applying SMC control
 *****************************************************************************/
double SMCSimulation::update(double setpoint, double dt)
{
    /*===========================================================================*
     * Sliding surface calculation
     *===========================================================================*/
    double error = setpoint - y;

    double d_error = (error - prev_error) / dt;

    double s = error + (lambda * d_error);

    prev_error = error;

    /*===========================================================================*
     * Sliding Mode Control law
     *===========================================================================*/
    double u = kp * (s > 0.0 ? 1.0 : -1.0);

    /*===========================================================================*
     * Plant update
     *===========================================================================*/
    y = plant.update(u, dt);
    return y;
}
