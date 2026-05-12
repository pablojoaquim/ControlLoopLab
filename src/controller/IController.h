#ifndef ICONTROLLER_H
#define ICONTROLLER_H

/*===========================================================================*/
/**
 * @file IController.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Abstract interface for closed-loop controllers.
 *
 * All concrete controller implementations (PID, Fuzzy, etc.) must derive from
 * this interface and implement the compute() and reset() methods.
 *
 * Key details:
 *   - compute() takes a setpoint, a measured process variable, and a time step,
 *     and returns the controller output (control action).
 *   - reset() clears any internal state accumulated between calls (e.g.,
 *     integral terms, previous errors).
 *
 * @section ABBR ABBREVIATIONS:
 *   - PID  - Proportional–Integral–Derivative
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
 * @class      IController
 * @brief      Abstract base class for all closed-loop controller implementations.
 *
 * Provides a common interface to compute a control action from a setpoint and
 * a measured process variable, and to reset any accumulated internal state.
 ******************************************************************************/
class IController
{
public:
    /*****************************************************************************
     * @fn         compute
     * @brief      Computes the control output for the current time step.
     * @param[in]  setpoint     The desired target value for the process variable.
     * @param[in]  measurement  The current measured value of the process variable.
     * @param[in]  dt           Elapsed time since the last call, in seconds.
     * @return     The computed control action (output signal).
     ******************************************************************************/
    virtual double compute(double setpoint, double measurement, double dt) = 0;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets all internal state of the controller to its initial values.
     * @return     None.
     ******************************************************************************/
    virtual void reset() = 0;

    virtual ~IController() = default;
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* ICONTROLLER_H */
