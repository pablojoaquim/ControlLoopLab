#ifndef SMC_SIMULATION_H
#define SMC_SIMULATION_H

/*===========================================================================*/
/**
 * @file smc_simulation.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) simulation wrapper.
 *
 * SMCSimulation couples an IPlant instance with a sign-based sliding mode
 * control law.  The controller is intentionally minimal: it exposes a single
 * update() call that computes one closed-loop step and returns the new plant
 * output.
 *
 * Control law:
 *   error   = setpoint - y
 *   d_error = (error - prev_error) / dt
 *   s       = error + lambda * d_error     (sliding surface)
 *   u       = kp * sign(s)                 (bang-bang control)
 *   y       = plant->update(u, dt)
 *
 * Key details:
 *   - The plant is owned via std::shared_ptr, allowing the same plant
 *     instance to be observed externally without lifetime issues.
 *   - kp and lambda are set at construction and treated as fixed parameters.
 *   - No output saturation is applied here; the caller or the plant model is
 *     responsible for any clamping needed by the actuator.
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
 * Header Files (C++ only)
 *===========================================================================*/
#ifdef __cplusplus

#include <memory>
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
 * @class      SMCSimulation
 * @brief      One-step sliding mode control simulation over an IPlant model.
 *
 * Owns (shared) the plant and maintains the minimal state required by the
 * control law: the current output y and the previous error for the derivative
 * approximation.
 ******************************************************************************/
class SMCSimulation
{
public:
    /*****************************************************************************
     * @fn         SMCSimulation
     * @brief      Constructs the simulation with a plant and SMC parameters.
     * @param[in]  plant   Shared pointer to the plant model to control.
     * @param[in]  kp      Gain applied to sign(s) to produce the control output.
     * @param[in]  lambda  Sliding surface gradient: weight of d_error relative
     *                     to error.
     ******************************************************************************/
    SMCSimulation(std::shared_ptr<IPlant> plant, double kp, double lambda);

    /*****************************************************************************
     * @fn         update
     * @brief      Executes one closed-loop SMC step.
     * @param[in]  setpoint  Desired reference value.
     * @param[in]  dt        Elapsed time since the last call, in seconds.
     * @return     Current plant output after applying the SMC control signal.
     ******************************************************************************/
    double update(double setpoint, double dt);

private:
    std::shared_ptr<IPlant> plant_;     /**< Controlled plant model.               */

    double kp_;                         /**< SMC gain applied to sign(s).          */
    double lambda_;                     /**< Sliding surface gradient parameter.   */

    double y_;                          /**< Current plant output.                 */
    double prev_error_;                 /**< Error value from the previous step.   */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* SMC_SIMULATION_H */