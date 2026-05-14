
#ifndef STATE_H
#define STATE_H

class State
{

public:
    State() = default;
    State(double x_, double y_, double heading_ = 0)
        : x(x_), y(y_), heading(heading_), gridx(0), gridy(0), gheading(0) {}

    double x{};
    double y{};
    double heading{};
    double z{};

    double gridx{};
    double gridy{};
    double gheading{};
};

struct Circle
{
    Circle() = default;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
    double x{};
    double y{};
    double r{};
};

#endif // STATE_H
