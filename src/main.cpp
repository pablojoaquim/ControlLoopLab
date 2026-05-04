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
    FILE *gp = popen("gnuplot -persistent", "w");
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
    double simulation_time = 5.0;
    double setpoint = 1.0;

    /*===========================================================================*
     * Plant setup
     *===========================================================================*/
    // FirstOrderPlant plant(1.0, 1.0);         // Plant: K=1, tau=1
    SecondOrderPlant plant_pid(1.0, 4.0, 0.3);   // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    SecondOrderPlant plant_fuzzy(1.0, 4.0, 0.3); // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    SecondOrderPlant plant_smc(1.0, 4.0, 0.3);   // Plant: K=1, omega_n=4, psi=0.3 (underdamped)

    /*===========================================================================*
     * PID Controller setup
     *===========================================================================*/
    double kp = 2.0;
    double ki = 1.0;
    double kd = 0.1;
    PIDController pid(kp, ki, kd, -10, 10); // PID: Kp=2, Ki=1, Kd=0.1, output limits [-10, 10]
    double y_pid = 0.0;

    /*===========================================================================*
     * Fuzzy Controller setup
     *===========================================================================*/
    // Error: setpoint is 1.0, max expected error at startup = 1.0
    LinguisticVariable error("error", -1.5, 1.5);
    error.addFuzzySet("NB", std::make_shared<TriangularMembershipFunction>(-1.5, -1.0,  -0.5));
    error.addFuzzySet("NS", std::make_shared<TriangularMembershipFunction>(-1.0, -0.5,   0.0));
    error.addFuzzySet("ZE", std::make_shared<TriangularMembershipFunction>(-0.3,  0.0,   0.3));
    error.addFuzzySet("PS", std::make_shared<TriangularMembershipFunction>( 0.0,  0.5,   1.0));
    error.addFuzzySet("PB", std::make_shared<TriangularMembershipFunction>( 0.5,  1.0,   1.5));

    // d_error: with ω_n=4, expect rates up to ~8 at startup. Normalize by 8.
    // Compute as: double de_norm = clamp(de_raw / 8.0, -1.0, 1.0);
    LinguisticVariable d_error("d_error", -1.0, 1.0);
    d_error.addFuzzySet("NG", std::make_shared<TriangularMembershipFunction>(-1.0, -0.5,  0.0));
    d_error.addFuzzySet("ZE", std::make_shared<TriangularMembershipFunction>(-0.3,  0.0,  0.3));
    d_error.addFuzzySet("PG", std::make_shared<TriangularMembershipFunction>( 0.0,  0.5,  1.0));

    // Control: PID saturates near ±10, keep same limit
    LinguisticVariable control("control", -10.0, 10.0);
    control.addFuzzySet("NB", std::make_shared<TriangularMembershipFunction>(-10.0, -7.0, -3.0));
    control.addFuzzySet("NS", std::make_shared<TriangularMembershipFunction>( -5.0, -2.5,  0.0));
    control.addFuzzySet("ZE", std::make_shared<TriangularMembershipFunction>( -1.0,  0.0,  1.0));
    control.addFuzzySet("PS", std::make_shared<TriangularMembershipFunction>(  0.0,  2.5,  5.0));
    control.addFuzzySet("PB", std::make_shared<TriangularMembershipFunction>(  3.0,  7.0, 10.0));

    // Create fuzzy inference engine and add rules
    InferenceEngine engine;
    // PB error
    engine.addRule(FuzzyRule({{"error","PB"},{"d_error","NG"}}, {"control","PS"}));
    engine.addRule(FuzzyRule({{"error","PB"},{"d_error","ZE"}}, {"control","PB"}));
    engine.addRule(FuzzyRule({{"error","PB"},{"d_error","PG"}}, {"control","PB"}));
    // PS error
    engine.addRule(FuzzyRule({{"error","PS"},{"d_error","NG"}}, {"control","ZE"}));
    engine.addRule(FuzzyRule({{"error","PS"},{"d_error","ZE"}}, {"control","PS"}));
    engine.addRule(FuzzyRule({{"error","PS"},{"d_error","PG"}}, {"control","PB"}));
    // ZE error
    engine.addRule(FuzzyRule({{"error","ZE"},{"d_error","NG"}}, {"control","NS"}));
    engine.addRule(FuzzyRule({{"error","ZE"},{"d_error","ZE"}}, {"control","ZE"}));
    engine.addRule(FuzzyRule({{"error","ZE"},{"d_error","PG"}}, {"control","PS"}));
    // NS error
    engine.addRule(FuzzyRule({{"error","NS"},{"d_error","NG"}}, {"control","NB"}));
    engine.addRule(FuzzyRule({{"error","NS"},{"d_error","ZE"}}, {"control","NS"}));
    engine.addRule(FuzzyRule({{"error","NS"},{"d_error","PG"}}, {"control","ZE"}));
    // NB error
    engine.addRule(FuzzyRule({{"error","NB"},{"d_error","NG"}}, {"control","NB"}));
    engine.addRule(FuzzyRule({{"error","NB"},{"d_error","ZE"}}, {"control","NB"}));
    engine.addRule(FuzzyRule({{"error","NB"},{"d_error","PG"}}, {"control","NS"}));

    // Create fuzzy controller
    Defuzzifier defuzz;

    double y_fuzzy = 0.0;
    double prev_error = 0.0;

    /*===========================================================================*
     * SMC Controller setup
     *===========================================================================*/
    double kp_smc = 5.0; // Gain for sliding mode control
    double epsilon_smc = 0.1; // Boundary layer thickness for tanh approximation
    double y_smc = 0.0;
    
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
         * Compute PID control signal and update plant
         *===========================================================================*/
        double error_pid = setpoint - y_pid;
        double u_pid = pid.compute(setpoint, y_pid, dt);
        y_pid = plant_pid.update(u_pid, dt);

        /*===========================================================================*
        * Fuzzy control signal and update plant
        *===========================================================================*/
        double error_fuzzy = setpoint - y_fuzzy;
        double de_norm     = std::max(-1.0, std::min(1.0, (error_fuzzy - prev_error) / (dt * 8.0)));
        prev_error         = error_fuzzy;

        auto outputs = engine.infer({
            {"error",   error.fuzzify(error_fuzzy)},
            {"d_error", d_error.fuzzify(de_norm)}
        });

        y_fuzzy = plant_fuzzy.update(defuzz.defuzzify(outputs["control"], control), dt);

        /*===========================================================================*
         * SMC control signal
         *===========================================================================*/
        // Sliding surface: s = error + lambda * d_error
        double error_smc = setpoint - y_smc;
        double lambda = 5; // Tuning parameter for sliding surface
        double s      = error_smc + lambda * (error_smc - prev_error) / dt;
        double u_smc = std::clamp(kp_smc * tanh((s > 0.0 ? 1.0 : -1.0)/epsilon_smc), -10.0, 10.0);

        y_smc = plant_smc.update(u_smc, dt);

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
                     std::to_string(u_pid));

        // Debug output
        LOG_DEBUG(&logger,
                  "t=" + std::to_string(t) + "," +
                      " y=" + std::to_string(y_pid) + "," +
                      " u=" + std::to_string(u_pid));

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
