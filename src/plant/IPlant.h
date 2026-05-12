#ifndef IPLANT_H
#define IPLANT_H

/*===========================================================================*/
/**
 * @file IPlant.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Abstract interface for discrete-time plant models.
 *
 * All concrete plant implementations (first-order, second-order, etc.) must
 * derive from this interface and implement the update() and reset() methods.
 *
 * Key details:
 *   - update() advances the plant state by one time step dt given an input
 *     signal, and returns the current plant output.
 *   - reset() clears any internal state accumulated between calls (e.g.,
 *     integrator values, previous outputs).
 *
 * @section ABBR ABBREVIATIONS:
 *   - dt  - Discrete time step (seconds)
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
 * @class      IPlant
 * @brief      Abstract base class for all discrete-time plant model implementations.
 *
 * Provides a common interface to advance the plant state by one time step and
 * to reset any accumulated internal state.
 ******************************************************************************/
class IPlant
{
public:
    /*****************************************************************************
     * @fn         update
     * @brief      Advances the plant state by one time step and returns the output.
     * @param[in]  input  Input signal applied to the plant (e.g. control action).
     * @param[in]  dt     Elapsed time since the last call, in seconds.
     * @return     Current plant output after the state update.
     ******************************************************************************/
    virtual double update(double input, double dt) = 0;

    /*****************************************************************************
     * @fn         reset
     * @brief      Resets all internal state of the plant to its initial values.
     * @return     None.
     ******************************************************************************/
    virtual void reset() = 0;

    virtual ~IPlant() = default;
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* IPLANT_H */
