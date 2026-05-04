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
 * @section DESC DESCRIPTION: Fuzzy Logic module (Mamdani)
 */
/*==========================================================================*/

#ifdef __cplusplus

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>

/*===========================================================================*
 * Types
 *===========================================================================*/

using FuzzySetName = std::string;
using LinguisticVariableName = std::string;

enum class FuzzyOperator
{
    AND_MIN,
    AND_PRODUCT,
    OR_MAX
};

/*===========================================================================*
 * Membership Functions
 *===========================================================================*/

class MembershipFunction
{
public:
    virtual float computeMembership(float x) const = 0;
    virtual ~MembershipFunction() = default;
};

class TriangularMembershipFunction : public MembershipFunction
{
    float a, b, c;

public:
    TriangularMembershipFunction(float a, float b, float c)
        : a(a), b(b), c(c) {}

    float computeMembership(float x) const override
    {
        if (x <= a || x >= c) return 0.0f;
        if (x == b) return 1.0f;
        if (x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }
};

class TrapezoidalMembershipFunction : public MembershipFunction
{
    float a, b, c, d;

public:
    TrapezoidalMembershipFunction(float a, float b, float c, float d)
        : a(a), b(b), c(c), d(d) {}

    float computeMembership(float x) const override
    {
        if (x <= a || x >= d) return 0.0f;
        if (x >= b && x <= c) return 1.0f;

        if (x < b)
            return (b != a) ? (x - a) / (b - a) : 0.0f;

        return (d != c) ? (d - x) / (d - c) : 0.0f;
    }
};

/*===========================================================================*
 * Fuzzy Set
 *===========================================================================*/

class FuzzySet
{
    FuzzySetName name;
    std::shared_ptr<MembershipFunction> mf;

public:
    FuzzySet(const FuzzySetName& name,
             std::shared_ptr<MembershipFunction> mf)
        : name(name), mf(mf) {}

    const FuzzySetName& getName() const { return name; }

    double getMembership(double x) const
    {
        return mf->computeMembership(x);
    }
};

/*===========================================================================*
 * Linguistic Variable
 *===========================================================================*/

class LinguisticVariable
{
    LinguisticVariableName name;
    double minValue;
    double maxValue;

    std::vector<FuzzySet> fuzzySets;

public:
    LinguisticVariable(const LinguisticVariableName& name,
                       double minValue,
                       double maxValue)
        : name(name), minValue(minValue), maxValue(maxValue) {}

    const LinguisticVariableName& getName() const { return name; }
    double getMin() const { return minValue; }
    double getMax() const { return maxValue; }

    void addFuzzySet(const FuzzySetName& setName,
                     std::shared_ptr<MembershipFunction> mf)
    {
        fuzzySets.emplace_back(setName, mf);
    }

    const std::vector<FuzzySet>& getFuzzySets() const
    {
        return fuzzySets;
    }

    std::map<FuzzySetName, double> fuzzify(double input) const
    {
        std::map<FuzzySetName, double> values;

        for (const auto& fs : fuzzySets)
            values[fs.getName()] = fs.getMembership(input);

        return values;
    }
};

/*===========================================================================*
 * Fuzzy Rule
 *===========================================================================*/

class FuzzyRule
{
    std::vector<std::pair<LinguisticVariableName, FuzzySetName>> antecedents;
    std::pair<LinguisticVariableName, FuzzySetName> consequent;
    FuzzyOperator op;

public:
    FuzzyRule(
        const std::vector<std::pair<LinguisticVariableName, FuzzySetName>>& ant,
        const std::pair<LinguisticVariableName, FuzzySetName>& cons,
        FuzzyOperator op = FuzzyOperator::AND_MIN)
        : antecedents(ant), consequent(cons), op(op) {}

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
                case FuzzyOperator::AND_PRODUCT: result *= m; break;
                case FuzzyOperator::OR_MAX:      result = std::max(result, m); break;
            }
        }

        return result;
    }

    const auto& getConsequent() const { return consequent; }
};

/*===========================================================================*
 * Inference Engine
 *===========================================================================*/

class InferenceEngine
{
    std::vector<FuzzyRule> rules;

public:
    void addRule(const FuzzyRule& r)
    {
        rules.push_back(r);
    }

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

/*===========================================================================*
 * Defuzzifier (Centroid)
 *===========================================================================*/

class Defuzzifier
{
    double step;

public:
    explicit Defuzzifier(double step = 0.05)
        : step(step) {}

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

#endif /* FUZZY_CONTROLLER_H */
