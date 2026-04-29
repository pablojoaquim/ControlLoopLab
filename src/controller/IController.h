#pragma once

class IController {
public:
    virtual ~IController() = default;
    virtual double compute(double setpoint, double measurement, double dt) = 0;
    virtual void reset() = 0;
};
