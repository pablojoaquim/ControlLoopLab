/*===========================================================================*/
/**
 * @file FirstOrderPlant.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * First-order plant model implementation using Euler integration.
 *
 * @section ABBR ABBREVIATIONS:
 *   - K   - Static gain
 *   - tau - Time constant (s)
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
 * Header Files
 *===========================================================================*/
#include "FirstOrderPlant.h"

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Variables Definitions
 *===========================================================================*/

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/

/*===========================================================================*
 * Local Inline Function Definitions and Function-Like Macros
 *===========================================================================*/

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         FirstOrderPlant::FirstOrderPlant
 * Description  Constructs the plant with the given gain and time constant,
 *              and zeroes the output state variable.
 *****************************************************************************/
FirstOrderPlant::FirstOrderPlant(double K, double tau)
    : K_(K), tau_(tau), y_(0.0) {}

/*****************************************************************************
 * Name         FirstOrderPlant::update
 * Description  Advances the plant output by one time step using the Euler method.
 *              The governing equation is:
 *                dy/dt = (-y + K*u) / tau
 *****************************************************************************/
double FirstOrderPlant::update(double input, double dt)
{
    /* Slope at the current state (Euler method) */
    double m = (-y_ + K_ * input) / tau_;
    y_ = y_ + m * dt;
    return y_;
}

/*****************************************************************************
 * Name         FirstOrderPlant::reset
 * Description  Resets the output state variable to zero.
 *****************************************************************************/
void FirstOrderPlant::reset()
{
    y_ = 0.0;
}
