#ifndef FUZZY_SIMULATION_H
#define FUZZY_SIMULATION_H

/*===========================================================================*/
/**
 * @file fuzzy_simulation.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Fuzzy Logic Controller (FLC) simulation wrapper.
 *
 * FuzzySimulation couples:
 *   - an IPlant instance
 *   - an FuzzyController instance
 *
 * to perform discrete-time closed-loop simulation.
 *
 * The simulation object is intentionally lightweight and delegates all
 * controller behavior to FuzzyController.
 *
 * Control flow:
 *   u = controller.compute(setpoint, y, dt)
 *   y = plant->update(u, dt)
 *
 * Key details:
 *   - The plant is owned through std::shared_ptr.
 *   - Controller state and switching logic are encapsulated inside
 *     FuzzyController.
 *   - The simulation wrapper only manages plant interaction and current
 *     output storage.
 *
 * @section ABBR ABBREVIATIONS:
 *   - FLC  - Fuzzy Logic Controller
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
#include "FuzzyController.h"

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/

/*****************************************************************************
 * @class      FuzzySimulation
 * @brief      Closed-loop FLC simulation over an IPlant model.
 *
 * Owns a shared plant model and an FuzzyController instance, and maintains
 * the current plant output state.
 ******************************************************************************/
class FuzzySimulation
{
public:
    /*****************************************************************************
     * @fn         FuzzySimulation
     * @brief      Constructs the simulation with a plant and FuzzyController parameters.
     * @param[in]  plant       Shared pointer to the controlled plant model.
     * @param[in]  output_min  Minimum allowable controller output.
     * @param[in]  output_max  Maximum allowable controller output.
     ******************************************************************************/
    FuzzySimulation(std::shared_ptr<IPlant> plant, double output_min, double output_max);
    
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

    LinguisticVariable error_;      /**< Linguistic variable for error. */
    LinguisticVariable d_error_;    /**< Linguistic variable for error derivative. */
    LinguisticVariable control_;    /**< Linguistic variable for control output. */
    InferenceEngine engine_;        /**< Fuzzy inference engine with registered rules. */
    Defuzzifier defuzz_;            /**< Defuzzifier for computing crisp control output. */

    double y_;                      /**< Current plant output. */

    double prev_error_;             /**< Previous error value for derivative calculation. */
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* FUZZY_SIMULATION_H */