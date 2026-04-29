/*===========================================================================*/
/**
 * @file main.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Add a description here
 *
 * @section ABBR ABBREVIATIONS:
 *   - @todo List any abbreviations, precede each with a dash ('-').
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
#include <iostream>
#include <memory>
#include <string>
#include <cstring>
#include <cstdint>
#include <vector>
#include <thread>
#include <chrono>
#include <cstdlib>

#include "katas.h"
#include "FirstOrderPlant.h"
#include "SecondOrderPlant.h"
#include "PIDController.h"

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/
#define NDEBUG

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/
#define NumElems(arr) (sizeof(arr) / sizeof((arr)[0]))

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
extern "C"
{
}

/*****************************************************************************
 * @fn         main
 * @brief      The main entry point
 * @param [in] void
 * @return     0 -success, -1 -Error
 *****************************************************************************/
int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    std::cout << "=== Start ===" << std::endl;
    // Plant: K=1, tau=1
    // FirstOrderPlant plant(1.0, 1.0);

    SecondOrderPlant plant(1.0, 4.0, 0.3);

    // PID: tweak these later
    PIDController pid(2.0, 1.0, 0.1, -10, 10);

    double dt = 0.01;
    double simulation_time = 5.0;
    double setpoint = 1.0;

    double y = 0.0;

    std::cout << "time,setpoint,output,control_signal" << std::endl;

    for (double t = 0.0; t <= simulation_time; t += dt) {
        double u = pid.compute(setpoint, y, dt);
        y = plant.update(u, dt);

        std::cout << t << "," << setpoint << "," << y << "," << u << std::endl;
    }

    std::cout << "===  End  ===" << std::endl;

    std::system("gnuplot -persist -e \"filename='output.csv'\" tools/plot.gp");

    return 0;
}
