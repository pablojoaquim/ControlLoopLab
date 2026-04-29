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
#include "logger.h"
#include "logger_sink_stdout.h"
#include "logger_sink_file.h"

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

    /*===========================================================================*
     * Gnuplot setup
     *===========================================================================*/
    // Open a pipe to gnuplot
    FILE* gp = popen("gnuplot -persistent", "w");
    if (!gp)
    {
        printf("Error: could not open gnuplot\n");
        return -1;
    }

    // Gnuplot initial configuration
    fprintf(gp, "set title 'PID Real-Time Response'\n");
    fprintf(gp, "set xlabel 'Time'\n");
    fprintf(gp, "set ylabel 'Value'\n");
    fprintf(gp, "set grid\n");

    /*===========================================================================*
     * Logging setup
     *===========================================================================*/
    Logger logger;
    LoggerStdoutSink consoleLogger;
    LoggerFileSink fileLogger("output.csv", LogLevel::Info);

    logger.setLevel(LogLevel::Debug);
    logger.addSink(&consoleLogger);
    logger.addSink(&fileLogger);
    
    // Header CSV output
    LOG_INFO(&logger, "time,setpoint,output,control_signal");

    /*===========================================================================*
     * Simulation parameters
     *===========================================================================*/
    double dt = 0.01;
    double simulation_time = 5.0;
    double setpoint = 1.0;

    LOG_INFO(&logger, "Starting simulation");

    /*===========================================================================*
     * System setup
     *===========================================================================*/
    // FirstOrderPlant plant(1.0, 1.0);         // Plant: K=1, tau=1
    SecondOrderPlant plant(1.0, 4.0, 0.3);      // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    PIDController pid(2.0, 1.0, 0.1, -10, 10);  // PID: Kp=2, Ki=1, Kd=0.1, u_min=-10, u_max=10

    double y = 0.0;

    /*===========================================================================*
     * Start streaming plot
     *===========================================================================*/
    fprintf(gp, "plot '-' using 1:2 with lines title 'Setpoint', "
                "'-' using 1:2 with lines title 'Output'\n");

    /*===========================================================================*
     * Simulation loop
     *===========================================================================*/                
    for (double t = 0.0; t <= simulation_time; t += dt) {
        
        double u = pid.compute(setpoint, y, dt);
        y = plant.update(u, dt);

        // Stream setpoint
        fprintf(gp, "%f %f\n", t, setpoint);

        // Stream output
        fprintf(gp, "%f %f\n", t, y);

        // CSV output
        LOG_INFO(&logger,
            std::to_string(t) + "," +
            std::to_string(setpoint) + "," +
            std::to_string(y) + "," +
            std::to_string(u)
        );

        // Debug output
        LOG_DEBUG(&logger,
            "t=" + std::to_string(t) +
            " y=" + std::to_string(y) +
            " u=" + std::to_string(u)
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
    
    LOG_INFO(&logger, "Simulation finished");

    /*===========================================================================*
     * Gnuplot cleanup
     *===========================================================================*/
    fprintf(gp, "e\n"); // end dataset
    fflush(gp);
    pclose(gp);

    std::cout << "===  End  ===" << std::endl;

    return 0;
}
