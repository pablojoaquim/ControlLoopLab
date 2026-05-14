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
#include <cmath>

#include "FirstOrderPlant.h"
#include "SecondOrderPlant.h"
#include "PIDController.h"
#include "FuzzyController.h"
#include "logger.h"
#include "logger_sink_stdout.h"
#include "logger_sink_file.h"

#include "smc_simulation.h"
#include "pid_simulation.h"
#include "fuzzy_simulation.h"

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
     * Simulation parameters
     *===========================================================================*/
    double dt = 0.01;
    double simulation_time = 5.0;
    double output_min = 0.0;    // Minimum control output (e.g. no braking)
    double output_max = 10.0;   // Maximum control output (e.g. full braking)

    /*===========================================================================*
     * Gnuplot setup
     *===========================================================================*/
    // Open a pipe to gnuplot
    FILE *gp = popen("gnuplot -persistent", "w");
    if (!gp)
    {
        printf("Error: could not open gnuplot\n");
        return -1;
    }

    // Gnuplot initial configuration
    fprintf(gp, "set title 'Real-Time Response'\n");
    fprintf(gp, "set xlabel 'Time'\n");
    fprintf(gp, "set ylabel 'Value'\n");
    fprintf(gp, "set grid\n");

    // Set fixed y-range for better visualization (adjust as needed)
    fprintf(gp, "set yrange [-0.1:1.3]\n");
    fprintf(gp, "set xrange [0:%f]\n", simulation_time);

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
    LOG_INFO(&logger, "time,setpoint,y_pid,y_fuzzy,y_smc");

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
     * Plant setup
     *===========================================================================*/
    // More realistic brake plant: overdamped, ω_n~3, ψ~1.2
    std::shared_ptr<IPlant> plant_fuzzy = std::make_shared<SecondOrderPlant>(1.0, 3.0, 1.2);
    std::shared_ptr<IPlant> plant_pid = std::make_shared<SecondOrderPlant>(1.0, 3.0, 1.2);
    std::shared_ptr<IPlant> plant_smc = std::make_shared<SecondOrderPlant>(1.0, 3.0, 1.2);

    /*===========================================================================*
     * Controllers setup
     *===========================================================================*/
    double y_fuzzy = 0.0;
    double y_pid = 0.0;
    double y_smc = 0.0;

    // Create each simulation instance, which owns the plant and maintains the control state
    FuzzySimulation fuzzy_sim(plant_fuzzy, output_min, output_max);
    PIDSimulation pid_sim(plant_pid, output_min, output_max);
    SMCSimulation smc_sim(plant_smc, output_min, output_max);

    /*===========================================================================*
     * Buffers (sliding window) for plotting
     *===========================================================================*/
    std::vector<double> time;
    std::vector<double> setpoint_vec;
    std::vector<double> y_pid_vec;
    std::vector<double> y_fuzzy_vec;
    std::vector<double> y_smc_vec;

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
            if (key != '\0')
            {
                switch (key)
                {
                case 'q':
                    exit_requested = true;
                    break;
                }

                fflush(stdout);
                key = '\0';
            }
        }

        /*===========================================================================*
         * Update setpoint (simulate driver pressing/releasing brake pedal)
         *===========================================================================*/
        double setpoint;
        if (t < 0.5)
            setpoint = 0.0; // no braking
        else if (t < 1.5)
            setpoint = t - 0.5; // ramp up (driver pressing)
        else if (t < 3.0)
            setpoint = 1.0; // full brake hold
        else if (t < 4.0)
            setpoint = 1.0 - (t - 3.0); // ramp down (driver releasing)
        else
            setpoint = 0.0; // released

        /*===========================================================================*
         * Compute control signal and update plant
         *===========================================================================*/
        y_pid = pid_sim.update(setpoint, dt);
        y_fuzzy = fuzzy_sim.update(setpoint, dt);
        y_smc = smc_sim.update(setpoint, dt);

        /*===========================================================================*
         * Plotting
         *===========================================================================*/
        // Update buffers for plotting
        time.push_back(t);
        setpoint_vec.push_back(setpoint);
        y_pid_vec.push_back(y_pid);
        y_fuzzy_vec.push_back(y_fuzzy);
        y_smc_vec.push_back(y_smc);

        // Keep only the latest MAX_POINTS for plotting
        if (time.size() > MAX_POINTS)
        {
            time.erase(time.begin());
            setpoint_vec.erase(setpoint_vec.begin());
            y_pid_vec.erase(y_pid_vec.begin());
            y_fuzzy_vec.erase(y_fuzzy_vec.begin());
            y_smc_vec.erase(y_smc_vec.begin());
        }

        // Send data to gnuplot
        fprintf(gp,
                "plot '-' using 1:2 with lines title 'Setpoint', "
                "'-' using 1:2 with lines title 'PID Output', "
                "'-' using 1:2 with lines title 'Fuzzy Output', "
                "'-' using 1:2 with lines title 'SMC Output'\n");

        // Setpoint
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], setpoint_vec[i]);
        fprintf(gp, "e\n");

        // PID Output
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], y_pid_vec[i]);
        fprintf(gp, "e\n");

        // Fuzzy Output
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], y_fuzzy_vec[i]);
        fprintf(gp, "e\n");

        // SMC Output
        for (size_t i = 0; i < time.size(); ++i)
            fprintf(gp, "%f %f\n", time[i], y_smc_vec[i]);
        fprintf(gp, "e\n");

        fflush(gp);

        /*===========================================================================*
         * Logging
         *===========================================================================*/
        // CSV output
        LOG_INFO(&logger,
                 std::to_string(t) + "," +
                     std::to_string(setpoint) + "," +
                     std::to_string(y_pid) + "," +
                     std::to_string(y_fuzzy) + "," +
                     std::to_string(y_smc));

        // Debug output
        LOG_DEBUG(&logger,
                  "t=" + std::to_string(t) +
                      " setpoint=" + std::to_string(setpoint) +
                      " y_pid=" + std::to_string(y_pid) +
                      " y_fuzzy=" + std::to_string(y_fuzzy) +
                      " y_smc=" + std::to_string(y_smc));

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
