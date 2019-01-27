#include "constraint.h"
#include "constparameter.h"
Constraint::Constraint()
{}

/**
 * @brief Constraint::Constraint, constructor of the class, initialize the constraints class
 * @param t, time for which the constraint is valid
 * @param selfPos, position of robot's itself, which is valid for the constraint time, x,y without angular
 */
Constraint::Constraint(const double& t, const std::vector<double > &selfPos):
    m_t(t),
    m_selfPos(selfPos)
{
    m_N = horizon;
    m_T = sampleInterval;
}

/**
 * @brief Constraint::distanceNorm calculate the distance
 * @param state the location at Kth step
 * @param blocked_c the center point of the blocked cell from another car
 * @return the first part of the inequality
 */
double Constraint::distanceNorm(const std::vector<double> &state, const std::vector<double> &blocked_c)
{
    std::vector<double> result;
    for(size_t i=0; i<state.size(); i++)
    {
        result.push_back(state[i]-blocked_c[i]);
    }
    return (VectorHelper::getInfinityNorm(result));
}

/**
 * @brief Constraint::psiQ calculate the psi_q in the constraint
 * @return psi_q (only use the upper bound, for simplicity)
 */
double Constraint::psiQ()
{
    ///minimun cell size
    double m_c_lower = cellWidthLower;
    double psi = std::max(minDistance, m_c_lower) + cellWidth/2;
    double psi_upper = psi + m_c_lower;
    return psi_upper;
}

/**
 * @brief Constraint::wrapConstraintFunctionObject wrapper object to use this in the nlopt object
 * @param u, control
 * @param grad gradient
 * @param data
 * @return
 */
double Constraint::wrapConstraintFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data)
{
    return (*reinterpret_cast<Constraint*>(data)) (u, grad);
}

/**
 * @brief Constraint::constraintTime returns the time instant where the constraint is to be examined
 * @return
 */
double Constraint::constraintTime() const
{
    return m_t;
}

/**
 * @brief Constraint::setActualSystem sets the actual system for which the constraint should be applied
 * @param model the current model of the robot, which should obey this constraint (p is the robot, the centerpoint to obey (avoid) is then q)
 * @param t0 current simulation time
 */
void Constraint::setActualSystem(const std::shared_ptr<Model> &model, const double& t0)
{
    m_model = model;
    m_t0 = t0;
}

/**
 * @brief Constraint::operator() calculate here the constraints in form of ||position_of_other_car - own_position||
 * @param u, control
 * @param grad
 * @param f_data
 * @return the inequality needs to be optimized
 */
double Constraint::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data)
{
    ///1. calculate the current trajectory of the referred system
    std::vector<std::vector<double> > trajectory;
    trajectory.push_back(m_model->start());
    std::vector<std::vector<double> > uReshaped = VectorHelper::reshapeXd(u, controlVecSize);

    for (size_t i = 0; i < m_N; i++)
    {
        trajectory.push_back(m_model->calcNext(trajectory.back(), uReshaped.at(i)));
    }
    for (auto it = trajectory.begin(); it != trajectory.end(); it++)
    {
        (*it).pop_back(); ///erase the angular position
    }

    ///2. get the correct position of the constraint
    int pos = (m_t - m_t0) / (double)(m_T);
    Q_ASSERT_X(pos >= 0, typeid(this).name() ,"pos is negative");

    ///3. calculate the constraint
    Cell cl;
    std::vector<size_t> tempo = cl.mapContinuousPositionToCells(m_selfPos);
    double inequality = distanceNorm(trajectory.at(pos), cl.mapCellsToCenterPoint(tempo));
    inequality = inequality - psiQ();
    inequality = -1.0 * inequality;

    ///4. send the inequality
    return inequality;
}

QDataStream& operator<<(QDataStream& out, const Constraint& constraint)
{
    out << (quint16)constraint.m_x0.size();
    for (size_t i = 0; i < constraint.m_x0.size(); i++)
    {
        out << constraint.m_x0.at(i);
    }
    out << constraint.m_t0;
    out << constraint.m_t;
    out << (quint16) constraint.m_N;
    out << constraint.m_T;
    for (size_t i = 0; i < constraint.m_selfPos.size(); i++)
    {
        out << constraint.m_selfPos.at(i);
    }
    return out;
}

QDataStream& operator>>(QDataStream& in, Constraint& constraint)
{
    quint16 xSize = 0;
    in >> xSize;
    constraint = Constraint();
    std::vector<double> x0;
    for (size_t i = 0; i < xSize; i++)
    {
        double xCur;
        in >> xCur;
        x0.push_back(xCur);
    }
    constraint.m_x0 = x0;
    double t, t0, T;
    in >> t;
    constraint.m_t = t;
    in >> t0;
    constraint.m_t0 = t0;
    quint16 N;
    in >> N;
    constraint.m_N = N;
    in >> T;
    constraint.m_T = T;
    std::vector<double> otherPos;
    for (size_t i = 0; i < xSize; i++)
    {
        double posCur;
        in >> posCur;
        otherPos.push_back(posCur);
    }
    constraint.m_selfPos = otherPos;
    return in;
}
