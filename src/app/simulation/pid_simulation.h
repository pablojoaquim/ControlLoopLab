#ifndef PID_SIMULATION_H
#define PID_SIMULATION_H

/*===========================================================================*/
/**
 * @file pid_simulation.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) simulation wrapper.
 *
 * @section ABBR ABBREVIATIONS:
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
#include "PIDController.h"

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
 * @class      PIDSimulation
 * @brief      One-step PID control simulation over an IPlant model.
 *
 * Owns (shared) the plant and maintains the minimal state required by the
 * control law: the current output y and the previous error for the derivative
 * approximation.
 ******************************************************************************/
class PIDSimulation
{
public:
    /*****************************************************************************
     * @fn         PIDSimulation
     * @brief      Constructs the simulation with a plant and PID parameters.
     * @param[in]  plant   Shared pointer to the plant model to control.
     * @param[in]  kp      Proportional gain.
     * @param[in]  ki      Integral gain.
     * @param[in]  kd      Derivative gain.
     * @param[in]  output_min  Minimum allowed control output.
     * @param[in]  output_max  Maximum allowed control output.
     ******************************************************************************/
    PIDSimulation(std::shared_ptr<IPlant> plant, double kp, double ki, double kd, double output_min, double output_max);

    /*****************************************************************************
     * @fn         update
     * @brief      Executes one closed-loop PID step.
     * @param[in]  setpoint  Desired reference value.
     * @param[in]  dt        Elapsed time since the last call, in seconds.
     * @return     Current plant output after applying the PID control signal.
     ******************************************************************************/
    double update(double setpoint, double dt);

private:
    std::shared_ptr<IPlant> plant_;     /**< Controlled plant model.               */

    double y_;                          /**< Current plant output.                 */
    double prev_error_;                 /**< Error value from the previous step.   */

    PIDController pid_;                 /**< PID controller instance.              */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* PID_SIMULATION_H */