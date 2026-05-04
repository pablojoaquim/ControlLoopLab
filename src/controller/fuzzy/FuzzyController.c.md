## The following is an example of how to use the FuzzyController.h lib

LinguisticVariable error("error", -1.0, 1.0);
error.addFuzzySet("NEG", std::make_shared<TriangularMembershipFunction>(-1, -1, 0));
error.addFuzzySet("ZERO", std::make_shared<TriangularMembershipFunction>(-0.5, 0, 0.5));
error.addFuzzySet("POS", std::make_shared<TriangularMembershipFunction>(0, 1, 1));

LinguisticVariable control("control", -10, 10);
control.addFuzzySet("LOW", std::make_shared<TriangularMembershipFunction>(-10, -5, 0));
control.addFuzzySet("MID", std::make_shared<TriangularMembershipFunction>(-2, 0, 2));
control.addFuzzySet("HIGH", std::make_shared<TriangularMembershipFunction>(0, 5, 10));

InferenceEngine engine;

engine.addRule({ {{"error","NEG"}}, {"control","HIGH"} });
engine.addRule({ {{"error","ZERO"}}, {"control","MID"} });
engine.addRule({ {{"error","POS"}}, {"control","LOW"} });

auto fuzzified = error.fuzzify(current_error);

std::map<std::string, std::map<std::string,double>> inputs;
inputs["error"] = fuzzified;

auto fuzzyOutput = engine.infer(inputs);

Defuzzifier defuzz;
double u = defuzz.defuzzify(fuzzyOutput["control"], control);
