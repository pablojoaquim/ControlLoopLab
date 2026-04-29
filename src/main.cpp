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
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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
     * Terminal setup for non-blocking input
     *===========================================================================*/    
    termios oldt, newt;
    // Save current terminal state
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo for non-blocking input
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Non-blocking
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    
    char key = '\0';
    bool exit_requested = false;

    /*===========================================================================*
     * Simulation parameters
     *===========================================================================*/
    double dt = 0.01;
    double simulation_time = 15.0;
    double setpoint = 1.0;
    double kp = 2.0;
    double ki = 1.0;
    double kd = 0.1;

    /*===========================================================================*
     * System setup
     *===========================================================================*/
    // FirstOrderPlant plant(1.0, 1.0);         // Plant: K=1, tau=1
    SecondOrderPlant plant(1.0, 4.0, 0.3);      // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    PIDController pid(kp, ki, kd, -10, 10);     // PID: Kp=2, Ki=1, Kd=0.1, output limits [-10, 10]

    double y = 0.0;

    /*===========================================================================*
     * Buffers (sliding window) for plotting
     *===========================================================================*/
    std::vector<double> time;
    std::vector<double> output;
    std::vector<double> sp;

    const size_t MAX_POINTS = 500;

    /*===========================================================================*
     * Simulation loop
     *===========================================================================*/                
    for (double t = 0.0; t <= simulation_time && !exit_requested; t += dt) 
    {
        /*===========================================================================*
         * Handle user input to adjust PID parameters and setpoint
         *===========================================================================*/  
        while (read(STDIN_FILENO, &key, 1) > 0)
        {
            if(key != '\0')
            {
                switch (key)
                {
                    case 'w': kp += 0.1; break;
                    case 's': kp -= 0.1; break;
                    case 'e': ki += 0.1; break;
                    case 'd': ki -= 0.1; break;
                    case 'r': kd += 0.01; break;
                    case 'f': kd -= 0.01; break;
                    case 't': setpoint += 0.5; break;
                    case 'g': setpoint -= 0.5; break;
                    case 'q': exit_requested = true; break;
                }

                printf("\rKp=%.2f Ki=%.2f Kd=%.2f SP=%.2f   ", kp, ki, kd, setpoint);
                fflush(stdout);

                pid.setKp(kp);
                pid.setKi(ki);
                pid.setKd(kd);

                key = '\0';
            }
        }
                
        /*===========================================================================*
         * Compute control signal and update plant 
         *===========================================================================*/  
        double u = pid.compute(setpoint, y, dt);
        y = plant.update(u, dt);

        /*===========================================================================*
         * Plotting
         *===========================================================================*/
        // Update buffers for plotting
        time.push_back(t);
        output.push_back(y);
        sp.push_back(setpoint);

        // Keep only the latest MAX_POINTS for plotting
        if (time.size() > MAX_POINTS)
        {
            time.erase(time.begin());
            output.erase(output.begin());
            sp.erase(sp.begin());
        }

        // Send data to gnuplot
        fprintf(gp,
            "plot '-' using 1:2 with lines title 'Setpoint', "
            "'-' using 1:2 with lines title 'Output'\n");

        // Setpoint
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], sp[i]);
        fprintf(gp, "e\n");

        // Output
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], output[i]);
        fprintf(gp, "e\n");

        fflush(gp);

        /*===========================================================================*
         * Logging
         *===========================================================================*/
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

        // Sleep for a short time to simulate real-time and allow gnuplot to update
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    /*===========================================================================*
     * Gnuplot cleanup
     *===========================================================================*/
    pclose(gp);

    /*===========================================================================*
     * Terminal cleanup
     *===========================================================================*/
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);


    std::cout << "===  End  ===" << std::endl;

    return 0;
}
