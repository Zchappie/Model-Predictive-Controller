#include "controlconstraint.h"
#include "constparameter.h"

/**
 * @brief ControlConstraint::ControlConstraint, constructor of the ConstrolConstraint class
 * @param t, time instant, which is valid for the constraint
 */
ControlConstraint::ControlConstraint(const double& t):
    m_t(t),
    m_T(sampleInterval)
{
}

/**
 * @brief setTime, set the time for valid control constraint
 * @param t0
 */
void ControlConstraint::setTime(const double& t0)
{
    m_t0 = t0;
}

/**
 * @brief ControlConstraint::operator () form control constraints
 * @param u, control
 * @param grad
 * @param f_data
 * @return
 */
double ControlConstraint::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data)
{
    int pos = std::floor((m_t - m_t0) / (m_T));
    double controlConstraints = std::sqrt(std::pow(u.at(pos*2),2) + std::pow(u.at(pos*2+1), 2)) - controlConstraintLimit;
    return controlConstraints;
}


/**
 * @brief ControlConstraint::wrapConstraintFunctionObject wrapper object to use this in the nlopt object
 * @param u
 * @param grad gradient
 * @param data
 * @return
 */
double ControlConstraint::wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data)
{
    return (*reinterpret_cast<ControlConstraint*>(data)) (u, grad);
}
