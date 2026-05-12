#ifndef SECOND_ORDER_PLANT_H
#define SECOND_ORDER_PLANT_H

/*===========================================================================*/
/**
 * @file SecondOrderPlant.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Second-order plant model with Euler integration.
 *
 * Implements the IPlant interface to simulate a second-order linear system
 * expressed in state-space form with two state variables:
 *
 *   x1 = y           (output / position)
 *   x2 = dy/dt       (velocity)
 *
 * The equations of motion are:
 *   dx1/dt = x2
 *   dx2/dt = -2 * psi * omega_n * x2 - omega_n^2 * x1 + K * omega_n^2 * u
 *
 * Key details:
 *   - update() evaluates both state derivatives and advances x1 and x2 by
 *     one Euler step of size dt, returning x1 as the plant output.
 *   - reset() zeroes both state variables, returning the plant to equilibrium.
 *   - K, omega_n, and psi are set at construction and remain fixed during
 *     simulation.
 *
 * @section ABBR ABBREVIATIONS:
 *   - K       - Static gain
 *   - omega_n - Undamped natural frequency (rad/s)
 *   - psi     - Damping ratio
 *   - dt      - Discrete time step (s)
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
 * @class      SecondOrderPlant
 * @brief      Discrete-time second-order plant model using the Euler method.
 *
 * Derives from IPlant and simulates the transfer function
 * K * omega_n^2 / (s^2 + 2*psi*omega_n*s + omega_n^2).
 ******************************************************************************/
class SecondOrderPlant : public IPlant
{
public:
    /*****************************************************************************
     * @fn         SecondOrderPlant
     * @brief      Constructs the plant with the given dynamic parameters.
     * @param[in]  K        Static gain of the plant.
     * @param[in]  omega_n  Undamped natural frequency, in rad/s.
     * @param[in]  psi      Damping ratio (0 = undamped, 1 = critically damped).
     ******************************************************************************/
    SecondOrderPlant(double K, double omega_n, double psi);

    /*****************************************************************************
     * @fn         update
     * @brief      Advances both state variables by one Euler time step.
     * @param[in]  input  Input signal applied to the plant.
     * @param[in]  dt     Elapsed time since the last call, in seconds.
     * @return     Updated plant output x1 (position / y).
     ******************************************************************************/
    double update(double input, double dt) override;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets both state variables to zero.
     * @return     None.
     ******************************************************************************/
    void reset() override;

private:
    double K_;       /**< Static gain. */
    double omega_n_; /**< Undamped natural frequency (rad/s). */
    double psi_;     /**< Damping ratio. */

    double x1_;      /**< Output state: y (position). */
    double x2_;      /**< Velocity state: dy/dt. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* SECOND_ORDER_PLANT_H */
