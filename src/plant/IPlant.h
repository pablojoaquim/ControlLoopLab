#pragma once

class IPlant {
public:
    virtual ~IPlant() = default;
    virtual double update(double input, double dt) = 0;
    virtual void reset() = 0;
};

