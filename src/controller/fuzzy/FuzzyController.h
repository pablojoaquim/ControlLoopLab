#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

/*===========================================================================*/
/**
 * @file fuzzy_controller.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2026 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Mamdani Fuzzy Logic controller implementation.
 *
 * Provides a complete fuzzy inference pipeline composed of the following stages:
 *   - Fuzzification: maps crisp input values to membership degrees across
 *     all fuzzy sets of each linguistic variable (LinguisticVariable::fuzzify).
 *   - Rule evaluation: each FuzzyRule combines its antecedent membership values
 *     using the chosen FuzzyOperator (AND_MIN, AND_PRODUCT, or OR_MAX) and clips
 *     the consequent fuzzy set at the resulting activation level.
 *   - Aggregation: InferenceEngine accumulates rule activations per output set,
 *     keeping the maximum activation for each (max aggregation).
 *   - Defuzzification: Defuzzifier applies the centroid method by numerically
 *     integrating over the output variable's universe of discourse.
 *
 * Membership function shapes supported:
 *   - Triangular  (TriangularMembershipFunction)
 *   - Trapezoidal (TrapezoidalMembershipFunction)
 *
 * Key details:
 *   - All classes are header-only (C++ only).
 *   - Membership functions are owned via std::shared_ptr to allow sharing
 *     across multiple fuzzy sets without duplication.
 *   - The defuzzification step size is configurable at construction time
 *     (default 0.05) to trade off accuracy against computation cost.
 *
 * @section ABBR ABBREVIATIONS:
 *   - MF   - Membership Function
 *   - LV   - Linguistic Variable
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

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/

/** Alias for the name string of a fuzzy set. */
using FuzzySetName = std::string;

/** Alias for the name string of a linguistic variable. */
using LinguisticVariableName = std::string;

/*****************************************************************************
 * @enum       FuzzyOperator
 * @brief      Logical operators used to combine antecedents inside a FuzzyRule.
 *
 * @var FuzzyOperator::AND_MIN
 *      Conjunction using the minimum of the membership values.
 * @var FuzzyOperator::AND_PRODUCT
 *      Conjunction using the algebraic product of the membership values.
 * @var FuzzyOperator::OR_MAX
 *      Disjunction using the maximum of the membership values.
 ******************************************************************************/
enum class FuzzyOperator
{
    AND_MIN,
    AND_PRODUCT,
    OR_MAX
};

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/

/*---------------------------------------------------------------------------*
 * Membership Functions
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      MembershipFunction
 * @brief      Abstract base class for all membership function shapes.
 ******************************************************************************/
class MembershipFunction
{
public:
    /*****************************************************************************
     * @fn         computeMembership
     * @brief      Computes the degree of membership for the crisp value x.
     * @param[in]  x  Crisp input value.
     * @return     Membership degree in the range [0, 1].
     ******************************************************************************/
    virtual float computeMembership(float x) const = 0;

    virtual ~MembershipFunction() = default;
};

/*****************************************************************************
 * @class      TriangularMembershipFunction
 * @brief      Triangular membership function defined by three points: a, b, c.
 *
 * The function rises linearly from 0 at x=a to 1 at x=b, then falls linearly
 * back to 0 at x=c. Returns 0 outside the interval [a, c].
 ******************************************************************************/
class TriangularMembershipFunction : public MembershipFunction
{
    float a, b, c;

public:
    /*****************************************************************************
     * @fn         TriangularMembershipFunction
     * @brief      Constructs the triangular MF with the given breakpoints.
     * @param[in]  a  Left foot (membership = 0).
     * @param[in]  b  Peak (membership = 1).
     * @param[in]  c  Right foot (membership = 0).
     ******************************************************************************/
    TriangularMembershipFunction(float a, float b, float c)
        : a(a), b(b), c(c) {}

    float computeMembership(float x) const override
    {
        if (x <= a || x >= c) return 0.0f;
        if (x == b)           return 1.0f;
        if (x < b)            return (x - a) / (b - a);
        return (c - x) / (c - b);
    }
};

/*****************************************************************************
 * @class      TrapezoidalMembershipFunction
 * @brief      Trapezoidal membership function defined by four points: a, b, c, d.
 *
 * The function rises linearly from 0 at x=a to 1 at x=b, remains 1 over
 * [b, c], then falls linearly back to 0 at x=d. Returns 0 outside [a, d].
 ******************************************************************************/
class TrapezoidalMembershipFunction : public MembershipFunction
{
    float a, b, c, d;

public:
    /*****************************************************************************
     * @fn         TrapezoidalMembershipFunction
     * @brief      Constructs the trapezoidal MF with the given breakpoints.
     * @param[in]  a  Left foot (membership = 0).
     * @param[in]  b  Left shoulder (membership = 1).
     * @param[in]  c  Right shoulder (membership = 1).
     * @param[in]  d  Right foot (membership = 0).
     ******************************************************************************/
    TrapezoidalMembershipFunction(float a, float b, float c, float d)
        : a(a), b(b), c(c), d(d) {}

    float computeMembership(float x) const override
    {
        if (x <= a || x >= d)  return 0.0f;
        if (x >= b && x <= c)  return 1.0f;
        if (x < b) return (b != a) ? (x - a) / (b - a) : 0.0f;
        return     (d != c) ? (d - x) / (d - c) : 0.0f;
    }
};

/*---------------------------------------------------------------------------*
 * Fuzzy Set
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      FuzzySet
 * @brief      Associates a linguistic label with a membership function.
 *
 * A FuzzySet stores a name (e.g. "HOT", "COLD") and a shared pointer to a
 * MembershipFunction. getMembership() delegates evaluation to that function.
 ******************************************************************************/
class FuzzySet
{
    FuzzySetName name;
    std::shared_ptr<MembershipFunction> mf;

public:
    /*****************************************************************************
     * @fn         FuzzySet
     * @brief      Constructs a fuzzy set with the given name and membership function.
     * @param[in]  name  Linguistic label for this set.
     * @param[in]  mf    Shared pointer to the membership function.
     ******************************************************************************/
    FuzzySet(const FuzzySetName& name,
             std::shared_ptr<MembershipFunction> mf)
        : name(name), mf(mf) {}

    /*****************************************************************************
     * @fn         getName
     * @brief      Returns the linguistic label of this fuzzy set.
     * @return     Const reference to the set name string.
     ******************************************************************************/
    const FuzzySetName& getName() const { return name; }

    /*****************************************************************************
     * @fn         getMembership
     * @brief      Computes the membership degree for the crisp value x.
     * @param[in]  x  Crisp input value.
     * @return     Membership degree in the range [0, 1].
     ******************************************************************************/
    double getMembership(double x) const
    {
        return mf->computeMembership(x);
    }
};

/*---------------------------------------------------------------------------*
 * Linguistic Variable
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      LinguisticVariable
 * @brief      A named variable defined over a universe of discourse [min, max],
 *             partitioned by a collection of fuzzy sets.
 *
 * fuzzify() maps a crisp input to a map of {set name → membership degree}
 * for every fuzzy set registered with this variable.
 ******************************************************************************/
class LinguisticVariable
{
    LinguisticVariableName name;
    double minValue;
    double maxValue;
    std::vector<FuzzySet> fuzzySets;

public:
    /*****************************************************************************
     * @fn         LinguisticVariable
     * @brief      Constructs a linguistic variable with a name and value range.
     * @param[in]  name      Variable name (e.g. "temperature").
     * @param[in]  minValue  Lower bound of the universe of discourse.
     * @param[in]  maxValue  Upper bound of the universe of discourse.
     ******************************************************************************/
    LinguisticVariable(const LinguisticVariableName& name,
                       double minValue,
                       double maxValue)
        : name(name), minValue(minValue), maxValue(maxValue) {}

    /*****************************************************************************
     * @fn         getName
     * @brief      Returns the name of this linguistic variable.
     * @return     Const reference to the variable name string.
     ******************************************************************************/
    const LinguisticVariableName& getName() const { return name; }

    /*****************************************************************************
     * @fn         getMin
     * @brief      Returns the lower bound of the universe of discourse.
     * @return     Minimum value.
     ******************************************************************************/
    double getMin() const { return minValue; }

    /*****************************************************************************
     * @fn         getMax
     * @brief      Returns the upper bound of the universe of discourse.
     * @return     Maximum value.
     ******************************************************************************/
    double getMax() const { return maxValue; }

    /*****************************************************************************
     * @fn         addFuzzySet
     * @brief      Registers a new fuzzy set for this linguistic variable.
     * @param[in]  setName  Linguistic label for the new set.
     * @param[in]  mf       Shared pointer to the membership function.
     * @return     None.
     ******************************************************************************/
    void addFuzzySet(const FuzzySetName& setName,
                     std::shared_ptr<MembershipFunction> mf)
    {
        fuzzySets.emplace_back(setName, mf);
    }

    /*****************************************************************************
     * @fn         getFuzzySets
     * @brief      Returns the list of fuzzy sets registered with this variable.
     * @return     Const reference to the internal vector of FuzzySet objects.
     ******************************************************************************/
    const std::vector<FuzzySet>& getFuzzySets() const
    {
        return fuzzySets;
    }

    /*****************************************************************************
     * @fn         fuzzify
     * @brief      Maps a crisp input value to membership degrees for all sets.
     * @param[in]  input  Crisp value to fuzzify.
     * @return     Map of {fuzzy set name → membership degree} for every set.
     ******************************************************************************/
    std::map<FuzzySetName, double> fuzzify(double input) const
    {
        std::map<FuzzySetName, double> values;

        for (const auto& fs : fuzzySets)
            values[fs.getName()] = fs.getMembership(input);

        return values;
    }
};

/*---------------------------------------------------------------------------*
 * Fuzzy Rule
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      FuzzyRule
 * @brief      Represents a single IF–THEN fuzzy rule in a Mamdani system.
 *
 * A rule contains one or more antecedent (variable, set) pairs combined with a
 * FuzzyOperator, and a single consequent (variable, set) pair.
 * evaluate() returns the firing strength of the rule given the current fuzzified
 * input map.
 ******************************************************************************/
class FuzzyRule
{
    std::vector<std::pair<LinguisticVariableName, FuzzySetName>> antecedents;
    std::pair<LinguisticVariableName, FuzzySetName> consequent;
    FuzzyOperator op;

public:
    /*****************************************************************************
     * @fn         FuzzyRule
     * @brief      Constructs a fuzzy rule with the given antecedents, consequent,
     *             and combining operator.
     * @param[in]  ant   Vector of (variable name, set name) antecedent pairs.
     * @param[in]  cons  (variable name, set name) consequent pair.
     * @param[in]  op    Operator used to combine antecedents (default: AND_MIN).
     ******************************************************************************/
    FuzzyRule(
        const std::vector<std::pair<LinguisticVariableName, FuzzySetName>>& ant,
        const std::pair<LinguisticVariableName, FuzzySetName>& cons,
        FuzzyOperator op = FuzzyOperator::AND_MIN)
        : antecedents(ant), consequent(cons), op(op) {}

    /*****************************************************************************
     * @fn         evaluate
     * @brief      Evaluates the rule's firing strength from the fuzzified inputs.
     * @param[in]  inputs  Map of {variable name → {set name → membership degree}}.
     * @return     Activation (firing strength) of this rule in the range [0, 1].
     ******************************************************************************/
    double evaluate(
        const std::map<LinguisticVariableName,
        std::map<FuzzySetName, double>>& inputs) const
    {
        double result = (op == FuzzyOperator::OR_MAX) ? 0.0 : 1.0;

        for (const auto& ant : antecedents)
        {
            double m = 0.0;

            auto varIt = inputs.find(ant.first);
            if (varIt != inputs.end())
            {
                auto setIt = varIt->second.find(ant.second);
                if (setIt != varIt->second.end())
                    m = setIt->second;
            }

            switch (op)
            {
                case FuzzyOperator::AND_MIN:     result = std::min(result, m); break;
                case FuzzyOperator::AND_PRODUCT: result *= m;                  break;
                case FuzzyOperator::OR_MAX:      result = std::max(result, m); break;
            }
        }

        return result;
    }

    /*****************************************************************************
     * @fn         getConsequent
     * @brief      Returns the consequent (output variable name, set name) pair.
     * @return     Const reference to the consequent pair.
     ******************************************************************************/
    const auto& getConsequent() const { return consequent; }
};

/*---------------------------------------------------------------------------*
 * Inference Engine
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      InferenceEngine
 * @brief      Applies all registered fuzzy rules to a set of fuzzified inputs
 *             and returns the aggregated output membership activations.
 *
 * For each rule, the firing strength is computed and the consequent output set
 * is updated via max-aggregation (the highest activation for each set wins).
 * Rules with zero activation are skipped.
 ******************************************************************************/
class InferenceEngine
{
    std::vector<FuzzyRule> rules;

public:
    /*****************************************************************************
     * @fn         addRule
     * @brief      Registers a fuzzy rule with the inference engine.
     * @param[in]  r  The FuzzyRule instance to add.
     * @return     None.
     ******************************************************************************/
    void addRule(const FuzzyRule& r)
    {
        rules.push_back(r);
    }

    /*****************************************************************************
     * @fn         infer
     * @brief      Evaluates all rules and aggregates output activations.
     * @param[in]  inputs  Map of {variable name → {set name → membership degree}}.
     * @return     Map of {output variable name → {set name → activation level}}.
     ******************************************************************************/
    std::map<LinguisticVariableName,
             std::map<FuzzySetName, double>>
    infer(const std::map<LinguisticVariableName,
         std::map<FuzzySetName, double>>& inputs) const
    {
        std::map<LinguisticVariableName,
                 std::map<FuzzySetName, double>> outputs;

        for (const auto& rule : rules)
        {
            double activation = rule.evaluate(inputs);
            if (activation <= 0.0) continue;

            const auto& [var, set] = rule.getConsequent();
            outputs[var][set] = std::max(outputs[var][set], activation);
        }

        return outputs;
    }
};

/*---------------------------------------------------------------------------*
 * Defuzzifier (Centroid)
 *---------------------------------------------------------------------------*/

/*****************************************************************************
 * @class      Defuzzifier
 * @brief      Converts a set of fuzzy output activations to a crisp value using
 *             the centroid (centre of gravity) defuzzification method.
 *
 * For each sample point x over the output variable's universe of discourse,
 * the clipped membership is computed for every active set, and the maximum
 * across sets is used (max aggregation before integration).
 ******************************************************************************/
class Defuzzifier
{
    double step;

public:
    /*****************************************************************************
     * @fn         Defuzzifier
     * @brief      Constructs the defuzzifier with the given numerical integration step.
     * @param[in]  step  Sampling interval across the universe of discourse (default 0.05).
     ******************************************************************************/
    explicit Defuzzifier(double step = 0.05)
        : step(step) {}

    /*****************************************************************************
     * @fn         defuzzify
     * @brief      Computes the crisp centroid output from aggregated fuzzy activations.
     * @param[in]  fuzzyValues  Map of {set name → activation level} for the output variable.
     * @param[in]  var          The output LinguisticVariable providing range and fuzzy sets.
     * @return     Crisp defuzzified value; returns 0.0 if the total area is zero.
     ******************************************************************************/
    double defuzzify(
        const std::map<FuzzySetName, double>& fuzzyValues,
        const LinguisticVariable& var) const
    {
        double num = 0.0;
        double den = 0.0;

        for (double x = var.getMin(); x <= var.getMax(); x += step)
        {
            double mu = 0.0;

            for (const auto& set : var.getFuzzySets())
            {
                double activation = 0.0;

                auto it = fuzzyValues.find(set.getName());
                if (it != fuzzyValues.end())
                    activation = it->second;

                double clipped = std::min(activation, set.getMembership(x));
                mu = std::max(mu, clipped);
            }

            num += x * mu;
            den += mu;
        }

        return (den == 0.0) ? 0.0 : (num / den);
    }
};

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* FUZZY_CONTROLLER_H */
