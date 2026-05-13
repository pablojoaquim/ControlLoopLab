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
    // double setpoint = 1.0;

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
    LOG_INFO(&logger, "time,setpoint,y_pid,y_fuzzy,y_smc,u_pid");

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
    // FirstOrderPlant plant(1.0, 1.0);         // Plant: K=1, tau=1
    // SecondOrderPlant plant_pid(1.0, 4.0, 0.3);   // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    // SecondOrderPlant plant_fuzzy(1.0, 4.0, 0.3); // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    // SecondOrderPlant plant_smc(1.0, 4.0, 0.3);   // Plant: K=1, omega_n=4, psi=0.3 (underdamped)
    // More realistic brake plant: overdamped, ω_n~3, ψ~1.2
    SecondOrderPlant plant_pid(1.0, 3.0, 1.2);
    SecondOrderPlant plant_fuzzy(1.0, 3.0, 1.2);
    SecondOrderPlant plant_smc(1.0, 3.0, 1.2);

    /*===========================================================================*
     * PID Controller setup
     *===========================================================================*/
    double kp = 10.0;
    double ki = 3.0;
    double kd = 0.8;
    // Brakes only push, never pull — clamp the control output to positive only
    PIDController pid(kp, ki, kd, 0, 10); // PID: Kp=10, Ki=3.0, Kd=0.8, output limits [0, 10]
    double y_pid = 0.0;

    /*===========================================================================*
     * Fuzzy Controller setup
     *===========================================================================*/
    // Error: setpoint is 1.0, max expected error at startup = 1.0
    LinguisticVariable error("error", -1.5, 1.5);
    error.addFuzzySet("NegativeBig", std::make_shared<TriangularMembershipFunction>(-1.5, -1.0, -0.5));
    error.addFuzzySet("NegativeSmall", std::make_shared<TriangularMembershipFunction>(-1.0, -0.5, 0.0));
    error.addFuzzySet("Zero", std::make_shared<TriangularMembershipFunction>(-0.3, 0.0, 0.3));
    error.addFuzzySet("PositiveSmall", std::make_shared<TriangularMembershipFunction>(0.0, 0.5, 1.0));
    error.addFuzzySet("PositiveBig", std::make_shared<TriangularMembershipFunction>(0.5, 1.0, 1.5));

    LinguisticVariable d_error("d_error", -1.0, 1.0);
    d_error.addFuzzySet("Negative", std::make_shared<TriangularMembershipFunction>(-1.0, -0.5, 0.0));
    d_error.addFuzzySet("Zero", std::make_shared<TriangularMembershipFunction>(-0.3, 0.0, 0.3));
    d_error.addFuzzySet("Positive", std::make_shared<TriangularMembershipFunction>(0.0, 0.5, 1.0));

    LinguisticVariable control("control", -10.0, 10.0);
    control.addFuzzySet("NegativeBig", std::make_shared<TriangularMembershipFunction>(-10.0, -7.0, -3.0));
    control.addFuzzySet("NegativeSmall", std::make_shared<TriangularMembershipFunction>(-5.0, -2.5, 0.0));
    control.addFuzzySet("Zero", std::make_shared<TriangularMembershipFunction>(-1.0, 0.0, 1.0));
    control.addFuzzySet("PositiveSmall", std::make_shared<TriangularMembershipFunction>(0.0, 2.5, 5.0));
    control.addFuzzySet("PositiveBig", std::make_shared<TriangularMembershipFunction>(5.0, 10.0, 10.0));

    // Create fuzzy inference engine and add rules
    InferenceEngine engine;

    // Fuzzy rules based on typical control logic for braking:
    // If error is PositiveBig (far from setpoint) and d_error is Negative (approaching), apply PositiveSmall control
    // If error is PositiveBig and d_error is Zero, apply PositiveBig control
    // If error is PositiveBig and d_error is Positive (moving away), apply PositiveBig control
    engine.addRule(FuzzyRule({{"error", "PositiveBig"}, {"d_error", "Negative"}}, {"control", "PositiveSmall"}));
    engine.addRule(FuzzyRule({{"error", "PositiveBig"}, {"d_error", "Zero"}}, {"control", "PositiveBig"}));
    engine.addRule(FuzzyRule({{"error", "PositiveBig"}, {"d_error", "Positive"}}, {"control", "PositiveBig"}));
    
    // If error is PositiveSmall and d_error is Negative, apply Zero control (already approaching, no need to increase)
    // If error is PositiveSmall and d_error is Zero, apply PositiveSmall control (still need to increase a bit)
    // If error is PositiveSmall and d_error is Positive, apply PositiveBig control (moving away, need more braking)
    engine.addRule(FuzzyRule({{"error", "PositiveSmall"}, {"d_error", "Negative"}}, {"control", "Zero"}));
    engine.addRule(FuzzyRule({{"error", "PositiveSmall"}, {"d_error", "Zero"}}, {"control", "PositiveSmall"}));
    engine.addRule(FuzzyRule({{"error", "PositiveSmall"}, {"d_error", "Positive"}}, {"control", "PositiveBig"}));
    
    // If error is Zero and d_error is Negative, apply NegativeSmall control (already at setpoint but approaching, may need to reduce braking)
    // If error is Zero and d_error is Zero, apply Zero control (at setpoint, no change)
    // If error is Zero and d_error is Positive, apply PositiveSmall control (at setpoint but moving away, may need to increase braking)
    engine.addRule(FuzzyRule({{"error", "Zero"}, {"d_error", "Negative"}}, {"control", "NegativeSmall"}));
    engine.addRule(FuzzyRule({{"error", "Zero"}, {"d_error", "Zero"}}, {"control", "Zero"}));
    engine.addRule(FuzzyRule({{"error", "Zero"}, {"d_error", "Positive"}}, {"control", "PositiveSmall"}));
    
    // If error is NegativeSmall and d_error is Negative, apply NegativeBig control (far from setpoint but approaching, may need to reduce braking significantly)
    // If error is NegativeSmall and d_error is Zero, apply NegativeSmall control (close to setpoint, no change or slight reduction)
    // If error is NegativeSmall and d_error is Positive, apply Zero control (close to setpoint but moving away, may need to increase braking)
    engine.addRule(FuzzyRule({{"error", "NegativeSmall"}, {"d_error", "Negative"}}, {"control", "NegativeBig"}));
    engine.addRule(FuzzyRule({{"error", "NegativeSmall"}, {"d_error", "Zero"}}, {"control", "NegativeSmall"}));
    engine.addRule(FuzzyRule({{"error", "NegativeSmall"}, {"d_error", "Positive"}}, {"control", "Zero"}));
    
    // If error is NegativeBig and d_error is Negative, apply NegativeBig control (far from setpoint but approaching, may need to reduce braking significantly)
    // If error is NegativeBig and d_error is Zero, apply NegativeBig control (far from setpoint, no change)
    // If error is NegativeBig and d_error is Positive, apply NegativeSmall control (far from setpoint and moving away, may need to reduce braking but not as much since it's already far)
    engine.addRule(FuzzyRule({{"error", "NegativeBig"}, {"d_error", "Negative"}}, {"control", "NegativeBig"}));
    engine.addRule(FuzzyRule({{"error", "NegativeBig"}, {"d_error", "Zero"}}, {"control", "NegativeBig"}));
    engine.addRule(FuzzyRule({{"error", "NegativeBig"}, {"d_error", "Positive"}}, {"control", "NegativeSmall"}));

    // Create fuzzy controller
    Defuzzifier defuzz;

    double y_fuzzy = 0.0;
    double prev_error_fuzzy = 0.0;

    /*===========================================================================*
     * SMC Controller setup
     *===========================================================================*/
    double kp_smc = 5.0; // Gain for sliding mode control
    double lambda = 0.2; // Tuning parameter for sliding surface
    double y_smc = 0.0;
    SMCSimulation smc_sim(plant_smc, kp_smc, lambda);

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
         * Compute PID control signal and update plant
         *===========================================================================*/
        double u_pid = pid.compute(setpoint, y_pid, dt);
        y_pid = plant_pid.update(u_pid, dt);

        /*===========================================================================*
         * Fuzzy control signal and update plant
         *===========================================================================*/
        double error_fuzzy = setpoint - y_fuzzy;
        double de_norm = std::max(-1.0, std::min(1.0, (error_fuzzy - prev_error_fuzzy) / (dt * 10.0))); // Normalize by max expected rate (10.0) and clamp to [-1, 1]
        prev_error_fuzzy = error_fuzzy;

        auto outputs = engine.infer({{"error", error.fuzzify(error_fuzzy)},
                                     {"d_error", d_error.fuzzify(de_norm)}});

        double u_fuzzy = defuzz.defuzzify(outputs["control"], control);
        y_fuzzy = plant_fuzzy.update(std::max(0.0, u_fuzzy), dt);

        /*===========================================================================*
         * SMC control signal
         *===========================================================================*/
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
                     std::to_string(y_smc) + "," +
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
