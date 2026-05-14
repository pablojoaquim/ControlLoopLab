/*===========================================================================*/
/**
 * @file fuzzy_simulation.cpp
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Fuzzy Logic Controller (FLC) simulation wrapper implementation.
 *
 * This module connects an IPlant implementation with an FuzzyController
 * instance to perform closed-loop discrete-time simulation.
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
 * Header Files
 *===========================================================================*/
#include "fuzzy_simulation.h"

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/*****************************************************************************
 * Name         FuzzySimulation::FuzzySimulation
 * Description  Stores the shared plant pointer, constructs the FuzzyController,
 *              and initializes the simulated plant output to zero.
 *****************************************************************************/
FuzzySimulation::FuzzySimulation(std::shared_ptr<IPlant> plant, double output_min, double output_max)
    : plant_(std::move(plant)),
      error_("error", -1.5, 1.5),
      d_error_("d_error", -10.0, 10.0),
      control_("control", -1.0 * output_max, output_max),   // Control output range is symmetric around zero
      engine_(),
      defuzz_(),
      y_(0.0),
      prev_error_(0.0)
{
    // Initialize the fuzzy controller with appropriate parameters
    // Error: setpoint range is [0,1], so expected tracking error is roughly [-1.5,1.5]
    error_.addFuzzySet("NegativeBig",    std::make_shared<TriangularMembershipFunction>(-1.5, -1.0, -0.2));
    error_.addFuzzySet("NegativeSmall",  std::make_shared<TriangularMembershipFunction>(-0.5, -0.15,  0.0));
    error_.addFuzzySet("Zero",           std::make_shared<TriangularMembershipFunction>(-0.15, 0.0,  0.15));
    error_.addFuzzySet("PositiveSmall",  std::make_shared<TriangularMembershipFunction>( 0.0,  0.3,  0.8));
    error_.addFuzzySet("PositiveBig",    std::make_shared<TriangularMembershipFunction>( 0.5,  1.0,  1.5));

    // Derivative of error: expected range is roughly [-10, 10] based on max rate of change
    d_error_.addFuzzySet("Negative", std::make_shared<TriangularMembershipFunction>(-10.0, -2.0,  0.0));
    d_error_.addFuzzySet("Zero",     std::make_shared<TriangularMembershipFunction>(-0.5,  0.0,  0.5));
    d_error_.addFuzzySet("Positive", std::make_shared<TriangularMembershipFunction>( 0.0,  2.0,  10.0));

    // Brake actuator output universe [-10,10]
    control_.addFuzzySet("NegativeBig",     std::make_shared<TriangularMembershipFunction>(-10.0, -7.0, -4.0));
    control_.addFuzzySet("NegativeSmall",   std::make_shared<TriangularMembershipFunction>(-5.0, -2.0, 0.0));
    control_.addFuzzySet("Zero",            std::make_shared<TriangularMembershipFunction>(-0.5, 0.0, 0.5));
    control_.addFuzzySet("PositiveSmall",   std::make_shared<TriangularMembershipFunction>( 0.0, 2.0, 5.0));
    control_.addFuzzySet("PositiveBig",     std::make_shared<TriangularMembershipFunction>( 4.0, 7.0, 10.0));

    // Fuzzy rules based on smooth brake control behavior:
    engine_.addRule(FuzzyRule({{"error","PositiveBig"},   {"d_error","Negative"}}, {"control","PositiveBig"}));
    engine_.addRule(FuzzyRule({{"error","PositiveBig"},   {"d_error","Zero"}},     {"control","PositiveBig"}));
    engine_.addRule(FuzzyRule({{"error","PositiveBig"},   {"d_error","Positive"}}, {"control","PositiveBig"}));

    engine_.addRule(FuzzyRule({{"error","PositiveSmall"}, {"d_error","Negative"}}, {"control","PositiveSmall"}));
    engine_.addRule(FuzzyRule({{"error","PositiveSmall"}, {"d_error","Zero"}},     {"control","PositiveSmall"}));
    engine_.addRule(FuzzyRule({{"error","PositiveSmall"}, {"d_error","Positive"}}, {"control","PositiveSmall"}));

    engine_.addRule(FuzzyRule({{"error","Zero"},          {"d_error","Negative"}}, {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","Zero"},          {"d_error","Zero"}},     {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","Zero"},          {"d_error","Positive"}}, {"control","Zero"}));
 
    engine_.addRule(FuzzyRule({{"error","NegativeBig"},   {"d_error","Negative"}}, {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","NegativeBig"},   {"d_error","Zero"}},     {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","NegativeBig"},   {"d_error","Positive"}}, {"control","Zero"}));

    engine_.addRule(FuzzyRule({{"error","NegativeSmall"}, {"d_error","Negative"}}, {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","NegativeSmall"}, {"d_error","Zero"}},     {"control","Zero"}));
    engine_.addRule(FuzzyRule({{"error","NegativeSmall"}, {"d_error","Positive"}}, {"control","Zero"}));
}

/*****************************************************************************
 * Name         FuzzySimulation::update
 * Description  Executes one closed-loop simulation step:
 *                1. Computes the control action using the FuzzyController.
 *                2. Advances the plant model by one discrete time step.
 *                3. Returns the updated plant output.
 *****************************************************************************/
double FuzzySimulation::update(double setpoint, double dt)
{
    /*===========================================================================*
     * Controller evaluation
     *===========================================================================*/
    double error = setpoint - y_; // Compute the error for fuzzification
    double d_error = (error - prev_error_) / dt;
    prev_error_ = error;

    /* Fuzzify inputs and perform inference to get fuzzy control outputs */
    auto outputs = engine_.infer({{"error", error_.fuzzify(error)},
                                  {"d_error", d_error_.fuzzify(d_error)}});

    /* Defuzzify the control output to get a crisp control signal */
    double u = defuzz_.defuzzify(outputs["control"], control_);

    /*===========================================================================*
    * Plant update
    *===========================================================================*/
    y_ = plant_->update(u, dt);
    return y_;
}
