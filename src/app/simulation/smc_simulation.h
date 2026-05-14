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
 * SMCSimulation couples:
 *   - an IPlant instance
 *   - an SMCController instance
 *
 * to perform discrete-time closed-loop simulation.
 *
 * The simulation object is intentionally lightweight and delegates all
 * controller behavior to SMCController.
 *
 * Control flow:
 *   u = controller.compute(setpoint, y, dt)
 *   y = plant->update(u, dt)
 *
 * Key details:
 *   - The plant is owned through std::shared_ptr.
 *   - Controller state and switching logic are encapsulated inside
 *     SMCController.
 *   - The simulation wrapper only manages plant interaction and current
 *     output storage.
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
#include "SMCController.h"

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/

/*****************************************************************************
 * @class      SMCSimulation
 * @brief      Closed-loop SMC simulation over an IPlant model.
 *
 * Owns a shared plant model and an SMCController instance, and maintains
 * the current plant output state.
 ******************************************************************************/
class SMCSimulation
{
public:
    /*****************************************************************************
     * @fn         SMCSimulation
     * @brief      Constructs the simulation with a plant and SMC parameters.
     * @param[in]  plant       Shared pointer to the controlled plant model.
     * @param[in]  kp          Switching gain.
     * @param[in]  lambda      Sliding surface coefficient.
     * @param[in]  output_min  Minimum allowable controller output.
     * @param[in]  output_max  Maximum allowable controller output.
     ******************************************************************************/
    SMCSimulation(std::shared_ptr<IPlant> plant,
                  double kp,
                  double lambda,
                  double output_min,
                  double output_max);

    /*****************************************************************************
     * @fn         update
     * @brief      Executes one closed-loop simulation step.
     * @param[in]  setpoint  Desired reference value.
     * @param[in]  dt        Elapsed time since the last call, in seconds.
     * @return     Current plant output after applying the control signal.
     ******************************************************************************/
    double update(double setpoint, double dt);

private:
    std::shared_ptr<IPlant> plant_; /**< Controlled plant model. */

    SMCController controller_;      /**< Encapsulated SMC controller. */

    double y_;                      /**< Current plant output. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* SMC_SIMULATION_H */