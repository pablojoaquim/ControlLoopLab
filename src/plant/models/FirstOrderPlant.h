#ifndef FIRST_ORDER_PLANT_H
#define FIRST_ORDER_PLANT_H

/*===========================================================================*/
/**
 * @file FirstOrderPlant.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * First-order plant model with Euler integration.
 *
 * Implements the IPlant interface to simulate a first-order linear system
 * described by the differential equation:
 *
 *   tau * dy/dt = -y + K * u
 *
 * Key details:
 *   - update() computes the slope at the current state and advances the output
 *     y by one Euler step of size dt.
 *   - reset() zeroes the internal output state, allowing the plant to be
 *     restarted from equilibrium.
 *   - K and tau are set at construction and remain fixed during simulation.
 *
 * @section ABBR ABBREVIATIONS:
 *   - K   - Static gain
 *   - tau - Time constant (s)
 *   - dt  - Discrete time step (s)
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
 * Header Files (C++ only)
 *===========================================================================*/
#ifdef __cplusplus

#include "IPlant.h"

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/

/*****************************************************************************
 * @class      FirstOrderPlant
 * @brief      Discrete-time first-order plant model using the Euler method.
 *
 * Derives from IPlant and simulates the transfer function K / (tau*s + 1).
 ******************************************************************************/
class FirstOrderPlant : public IPlant
{
public:
    /*****************************************************************************
     * @fn         FirstOrderPlant
     * @brief      Constructs the plant with the given gain and time constant.
     * @param[in]  K    Static gain of the plant.
     * @param[in]  tau  Time constant of the plant, in seconds.
     ******************************************************************************/
    FirstOrderPlant(double K, double tau);

    /*****************************************************************************
     * @fn         update
     * @brief      Advances the plant output by one Euler time step.
     * @param[in]  input  Input signal applied to the plant.
     * @param[in]  dt     Elapsed time since the last call, in seconds.
     * @return     Updated plant output y.
     ******************************************************************************/
    double update(double input, double dt) override;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets the output state variable to zero.
     * @return     None.
     ******************************************************************************/
    void reset() override;

private:
    double K_;   /**< Static gain. */
    double tau_; /**< Time constant (s). */
    double y_;   /**< Current output state. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* FIRST_ORDER_PLANT_H */
