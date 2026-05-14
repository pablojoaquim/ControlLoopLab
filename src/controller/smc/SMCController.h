#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

/*===========================================================================*/
/**
 * @file SMCController.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Sliding Mode Controller (SMC) implementation.
 *
 * Implements the IController interface using a first-order sliding surface:
 *
 *      s = lambda * error + error_rate
 *
 * and a discontinuous switching control law:
 *
 *      u = kp * sign(s)
 *
 * Key details:
 *   - compute() evaluates the sliding surface and switching law.
 *   - Output is clamped to [u_min, u_max].
 *   - reset() clears the previous error state.
 *   - Parameters may be updated at runtime.
 *
 * @section ABBR ABBREVIATIONS:
 *   - SMC    - Sliding Mode Control
 *   - dt     - Discrete time step (seconds)
 *   - eta    - Switching gain
 *   - lambda - Sliding surface coefficient
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
 * Exported Classes
 *===========================================================================*/

/*****************************************************************************
 * @class      SMCController
 * @brief      Sliding Mode Controller with output saturation.
 ******************************************************************************/
class SMCController : public IController
{
public:
    /*****************************************************************************
     * @fn         SMCController
     * @brief      Constructs an SMC controller.
     * @param[in]  kp     Switching gain.
     * @param[in]  lambda  Sliding surface coefficient.
     * @param[in]  u_min   Minimum allowable controller output.
     * @param[in]  u_max   Maximum allowable controller output.
     ******************************************************************************/
    SMCController(double kp,
                  double lambda,
                  double u_min,
                  double u_max);

    /*****************************************************************************
     * @fn         compute
     * @brief      Computes the SMC control output.
     * @param[in]  setpoint     Desired target value.
     * @param[in]  measurement  Current measured process value.
     * @param[in]  dt           Elapsed time since the last call, in seconds.
     * @return     Clamped control output.
     ******************************************************************************/
    double compute(double setpoint,
                   double measurement,
                   double dt) override;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets the controller internal state.
     ******************************************************************************/
    void reset() override;

    /*****************************************************************************
     * @fn         setLambda
     * @brief      Updates the sliding surface coefficient.
     ******************************************************************************/
    void setLambda(double lambda);

    /*****************************************************************************
     * @fn         setKp
     * @brief      Updates the switching gain.
     ******************************************************************************/
    void setKp(double kp);

    /*****************************************************************************
     * @fn         getLambda
     * @brief      Returns the sliding surface coefficient.
     ******************************************************************************/
    double getLambda() const;

    /*****************************************************************************
     * @fn         getKp
     * @brief      Returns the switching gain.
     ******************************************************************************/
    double getKp() const;

private:
    double kp_;         /**< Switching gain. */
    double lambda_;     /**< Sliding surface coefficient. */

    double u_min_;      /**< Lower saturation limit. */
    double u_max_;      /**< Upper saturation limit. */

    double prev_error_; /**< Error from previous step. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* SMC_CONTROLLER_H */