#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/*===========================================================================*/
/**
 * @file PIDController.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Proportional–Integral–Derivative (PID) controller implementation.
 *
 * Implements the IController interface to provide a standard discrete-time PID
 * control law with output saturation (anti-windup via clamping).
 *
 * Key details:
 *   - compute() evaluates the PID equation on each call, using the elapsed
 *     time dt to integrate the error and approximate its derivative.
 *   - Output is clamped to [u_min, u_max] to prevent actuator saturation.
 *   - reset() zeroes the accumulated integral term and the previous error,
 *     allowing safe re-use of the controller after a mode change or startup.
 *   - Gains (kp, ki, kd) may be updated at runtime via the setter methods.
 *
 * @section ABBR ABBREVIATIONS:
 *   - PID  - Proportional–Integral–Derivative
 *   - kp   - Proportional gain
 *   - ki   - Integral gain
 *   - kd   - Derivative gain
 *   - dt   - Discrete time step (seconds)
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

#include "IController.h"

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
 * @class      PIDController
 * @brief      Discrete-time PID controller with output saturation.
 *
 * Derives from IController and implements the standard PID control law.
 * Output is clamped to the range [u_min, u_max] supplied at construction.
 ******************************************************************************/
class PIDController : public IController
{
public:
    /*****************************************************************************
     * @fn         PIDController
     * @brief      Constructs a PID controller with the given gains and output limits.
     * @param[in]  kp     Proportional gain.
     * @param[in]  ki     Integral gain.
     * @param[in]  kd     Derivative gain.
     * @param[in]  u_min  Minimum allowable controller output.
     * @param[in]  u_max  Maximum allowable controller output.
     ******************************************************************************/
    PIDController(double kp, double ki, double kd,
                  double u_min, double u_max);

    /*****************************************************************************
     * @fn         compute
     * @brief      Computes the PID control output for the current time step.
     * @param[in]  setpoint     The desired target value for the process variable.
     * @param[in]  measurement  The current measured value of the process variable.
     * @param[in]  dt           Elapsed time since the last call, in seconds.
     * @return     The clamped PID control action.
     ******************************************************************************/
    double compute(double setpoint, double measurement, double dt) override;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets the cumulative error and previous error to zero.
     * @return     None.
     ******************************************************************************/
    void reset() override;

    /*****************************************************************************
     * @fn         setKp
     * @brief      Updates the proportional gain at runtime.
     * @param[in]  kp  New proportional gain value.
     * @return     None.
     ******************************************************************************/
    void setKp(double kp);

    /*****************************************************************************
     * @fn         setKi
     * @brief      Updates the integral gain at runtime.
     * @param[in]  ki  New integral gain value.
     * @return     None.
     ******************************************************************************/
    void setKi(double ki);

    /*****************************************************************************
     * @fn         setKd
     * @brief      Updates the derivative gain at runtime.
     * @param[in]  kd  New derivative gain value.
     * @return     None.
     ******************************************************************************/
    void setKd(double kd);

    /*****************************************************************************
     * @fn         getKp
     * @brief      Returns the current proportional gain.
     * @return     Proportional gain value.
     ******************************************************************************/
    double getKp() const;

    /*****************************************************************************
     * @fn         getKi
     * @brief      Returns the current integral gain.
     * @return     Integral gain value.
     ******************************************************************************/
    double getKi() const;

    /*****************************************************************************
     * @fn         getKd
     * @brief      Returns the current derivative gain.
     * @return     Derivative gain value.
     ******************************************************************************/
    double getKd() const;

private:
    double kp_;               /**< Proportional gain. */
    double ki_;               /**< Integral gain. */
    double kd_;               /**< Derivative gain. */

    double u_min_;            /**< Lower bound for output clamping. */
    double u_max_;            /**< Upper bound for output clamping. */

    double cumulative_error_; /**< Accumulated integral term. */
    double prev_error_;       /**< Error value from the previous time step. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* PID_CONTROLLER_H */
