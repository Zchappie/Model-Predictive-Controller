#ifndef CONTROLCONSTRAINT_H
#define CONTROLCONSTRAINT_H

#include <vector>
#include <cmath>


/**
 * @brief The ControlConstraint class, set boundaries for the control commands
 */
class ControlConstraint
{

public:
    ControlConstraint(const double& t);
    static double wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    void setTime(const double& t0);

private:
    ///time instant, which is valid for the constraint
    double m_t;
    ///current global time
    double m_t0;
    ///time interval
    double m_T;
};

#endif // CONTROLCONSTRAINT_H
